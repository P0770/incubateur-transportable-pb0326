// =============================================================
//  PB-0326 — Incubateur transportable pour cellules
//  HEPIA HES-SO — Groupe 3 : Lyla Founas, Mattéo Pozzo di Borgo
//  Version : 2.0 — Mai 2026
// =============================================================

#include <TFT_eSPI.h>
#include <ESP32Servo.h>
#include <Wire.h>

// =============================================================
//  PINS
// =============================================================
#define BROCHE_SERVO        18
#define BROCHE_PELTIER_PWM  19
#define BROCHE_PELTIER_IN1  20
#define BROCHE_PELTIER_IN2  21
#define BROCHE_POMPE        22
#define BROCHE_LED_ROUGE    23
#define BROCHE_BOUTON_GO     5

// =============================================================
//  SERVO SG90 — Robinet CO2
// =============================================================
#define ANGLE_PURGE    0
#define ANGLE_CHARGE  90
#define ANGLE_INJECT 180

#define DUREE_PURGE    3000
#define DUREE_CHARGE  10000
#define DUREE_INJECT   2000

// =============================================================
//  CONSIGNES BIOLOGIQUES
// =============================================================
#define CONSIGNE_TEMP     37.0f
#define TEMP_SECURITE     40.0f
#define CONSIGNE_CO2       5.0f
#define TOLERANCE_CO2      0.3f
#define SEUIL_HR_MIN      95.0f
#define SEUIL_HR_MAX      99.0f

// =============================================================
//  PID PELTIER
// =============================================================
#define KP  15.0f
#define KI   0.5f
#define KD   2.0f

// =============================================================
//  TIMINGS
// =============================================================
#define INTERVALLE_CO2     5000
#define INTERVALLE_HR     30000
#define INTERVALLE_ECRAN   2000
#define DUREE_INIT         5000

// =============================================================
//  ÉTATS — simple entier qui s'incrémente
// =============================================================
#define ETAT_INIT            0
#define ETAT_CHECK_CAPTEURS  1
#define ETAT_ATTENTE_DONNEES 2
#define ETAT_BUFFER          3
#define ETAT_MARCHE          4
#define ETAT_ERREUR          5

// =============================================================
//  OBJETS
// =============================================================
TFT_eSPI ecran = TFT_eSPI();
Servo servo;

// =============================================================
//  VARIABLES
// =============================================================
int etat         = ETAT_INIT;
int etatPrecedent = -1;       // Force le 1er affichage

// Capteurs (stubs — remplacer par lecture réelle)
float temperature = 36.5f;
float humidite    = 97.0f;
float co2         =  5.0f;

// PID
float integrale   = 0.0f;
float erreurPrec  = 0.0f;
unsigned long tempsPID = 0;

// Timers
unsigned long dernierCO2   = 0;
unsigned long derniereHR   = 0;
unsigned long dernierEcran = 0;
unsigned long tempsInit    = 0;

// Flags
bool pompeActive  = false;
bool purgeInitFait = false;

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(115200);

  pinMode(BROCHE_PELTIER_IN1, OUTPUT);
  pinMode(BROCHE_PELTIER_IN2, OUTPUT);
  pinMode(BROCHE_POMPE,       OUTPUT);
  pinMode(BROCHE_LED_ROUGE,   OUTPUT);
  pinMode(BROCHE_BOUTON_GO,   INPUT_PULLUP);

  // PWM Peltier — canal 0, 5 kHz, 8 bits
  ledcSetup(0, 5000, 8);
  ledcAttachPin(BROCHE_PELTIER_PWM, 0);

  toutEteindre();

  servo.attach(BROCHE_SERVO);
  servo.write(ANGLE_CHARGE);

  Wire.begin();

  ecran.init();
  ecran.setRotation(1);
  ecran.fillScreen(TFT_BLACK);

  tempsInit = millis();
  Serial.println("[INIT] Démarrage");
}

// =============================================================
//  LOOP — MACHINE À ÉTATS
// =============================================================
void loop() {

  lireCapteurs();

  // Affichage uniquement si changement d'état
  if (etat != etatPrecedent) {
    afficherEcran(etat);
    etatPrecedent = etat;
    Serial.printf("[ETAT] %d\n", etat);
  }

  switch (etat) {

    // ---- 0 : INIT ----------------------------------------
    case ETAT_INIT:
      if (!purgeInitFait) {
        servo.write(ANGLE_PURGE);
        purgeInitFait = true;
      }
      if (millis() - tempsInit >= DUREE_INIT) {
        integrale    = 0;
        erreurPrec   = 0;
        pompeActive  = false;
        servo.write(ANGLE_CHARGE);
        etat++;          // → ETAT_CHECK_CAPTEURS
      }
      break;

    // ---- 1 : CHECK CAPTEURS ------------------------------
    case ETAT_CHECK_CAPTEURS:
      if (verifierCapteursI2C()) {
        etat++;          // → ETAT_ATTENTE_DONNEES
      } else {
        signalerErreur("Capteur absent");
      }
      break;

    // ---- 2 : ATTENTE DONNEES -----------------------------
    case ETAT_ATTENTE_DONNEES:
      if (temperature > 10.0f && temperature < 45.0f && humidite > 0.0f) {
        etat++;          // → ETAT_BUFFER
      } else {
        signalerErreur("Donnees invalides");
      }
      break;

    // ---- 3 : BUFFER (attente GO) -------------------------
    case ETAT_BUFFER:
      if (millis() - dernierEcran >= INTERVALLE_ECRAN) {
        afficherEcran(etat);
        dernierEcran = millis();
      }
      if (digitalRead(BROCHE_BOUTON_GO) == LOW) {
        delay(50); // anti-rebond
        if (digitalRead(BROCHE_BOUTON_GO) == LOW) {
          dernierCO2  = millis();
          derniereHR  = millis();
          tempsPID    = millis();
          etat++;      // → ETAT_MARCHE
        }
      }
      break;

    // ---- 4 : MARCHE --------------------------------------
    case ETAT_MARCHE:
      // Sécurité temperature
      if (temperature > TEMP_SECURITE) {
        signalerErreur("TEMP CRITIQUE");
        break;
      }

      // Regulation Peltier
      miseAJourPID();

      // Humidité toutes les 30s
      if (millis() - derniereHR >= INTERVALLE_HR) {
        miseAJourHumidite();
        derniereHR = millis();
      }

      // CO2 toutes les 5s
      if (millis() - dernierCO2 >= INTERVALLE_CO2) {
        miseAJourCO2();
        dernierCO2 = millis();
      }

      // Refresh écran toutes les 2s
      if (millis() - dernierEcran >= INTERVALLE_ECRAN) {
        afficherEcran(etat);
        dernierEcran = millis();
      }
      
      break;

    // ---- 5 : ERREUR --------------------------------------
    case ETAT_ERREUR:
      toutEteindre();
      // Blink LED rouge
      static unsigned long dernierBlink = 0;
      static bool ledActive = false;
      if (millis() - dernierBlink >= 500) {
        ledActive = !ledActive;
        digitalWrite(BROCHE_LED_ROUGE, ledActive);
        dernierBlink = millis();
      }
      break;
  }
}

// =============================================================
//  PID PELTIER — non bloquant
// =============================================================
void miseAJourPID() {
  unsigned long maintenant = millis();
  float dt = (maintenant - tempsPID) / 1000.0f;
  if (dt <= 0.0f || tempsPID == 0) { tempsPID = maintenant; return; }
  tempsPID = maintenant;

  float erreur  = CONSIGNE_TEMP - temperature;
  integrale    += erreur * dt;
  integrale     = constrain(integrale, -100.0f, 100.0f); // anti-windup
  float derivee = (erreur - erreurPrec) / dt;
  erreurPrec    = erreur;

  float sortie  = KP * erreur + KI * integrale + KD * derivee;
  sortie        = constrain(sortie, -255.0f, 255.0f);

  if (sortie > 0) {
    digitalWrite(BROCHE_PELTIER_IN1, HIGH);
    digitalWrite(BROCHE_PELTIER_IN2, LOW);
    ledcWrite(0, (int)sortie);
  } else if (sortie < 0) {
    digitalWrite(BROCHE_PELTIER_IN1, LOW);
    digitalWrite(BROCHE_PELTIER_IN2, HIGH);
    ledcWrite(0, (int)(-sortie));
  } else {
    digitalWrite(BROCHE_PELTIER_IN1, LOW);
    digitalWrite(BROCHE_PELTIER_IN2, LOW);
    ledcWrite(0, 0);
  }
}

// =============================================================
//  HUMIDITÉ — hystérésis pompe
// =============================================================
void miseAJourHumidite() {
  if (humidite < SEUIL_HR_MIN && !pompeActive) {
    digitalWrite(BROCHE_POMPE, HIGH);
    pompeActive = true;
    Serial.printf("[HR] Pompe ON — %.1f%%\n", humidite);
  } else if (humidite > SEUIL_HR_MAX && pompeActive) {
    digitalWrite(BROCHE_POMPE, LOW);
    pompeActive = false;
    Serial.printf("[HR] Pompe OFF — %.1f%%\n", humidite);
  }
}

// =============================================================
//  CO2 — cycle injection si hors tolérance
// =============================================================
void miseAJourCO2() {
  Serial.printf("[CO2] %.2f%%\n", co2);
  if (co2 < (CONSIGNE_CO2 - TOLERANCE_CO2) || co2 > (CONSIGNE_CO2 + TOLERANCE_CO2)) {
    injecterCO2();
  }
}

// =============================================================
//  INJECTION CO2 — Séquence SG90 (bloquante ~15s)
// =============================================================
void injecterCO2() {
  Serial.println("[CO2] PURGE");
  servo.write(ANGLE_PURGE);   delay(DUREE_PURGE);
  Serial.println("[CO2] CHARGE");
  servo.write(ANGLE_CHARGE);  delay(DUREE_CHARGE);
  Serial.println("[CO2] INJECT");
  servo.write(ANGLE_INJECT);  delay(DUREE_INJECT);
  servo.write(ANGLE_CHARGE);  // retour position neutre
  Serial.println("[CO2] Cycle terminé");
}

// =============================================================
//  CAPTEURS — STUBS (remplacer par lecture I2C réelle)
// =============================================================
void lireCapteurs() {
  // TODO : SHT35 → temperature + humidite
  // TODO : SCD40 → co2
  temperature = 36.5f + (random(-10, 10) / 100.0f);
  humidite    = 97.0f + (random(-5,  5)  / 10.0f);
  co2         =  5.0f + (random(-5,  5)  / 100.0f);
}

// =============================================================
//  CHECK I2C — scan capteurs
// =============================================================
bool verifierCapteursI2C() {
  bool sht35 = false, scd40 = false;
  Wire.beginTransmission(0x44); if (Wire.endTransmission() == 0) sht35 = true;
  Wire.beginTransmission(0x62); if (Wire.endTransmission() == 0) scd40 = true;
  Serial.printf("[I2C] SHT35:%s SCD40:%s\n", sht35?"OK":"--", scd40?"OK":"--");
  return true; // STUB — remplacer par : return sht35 && scd40;
}

// =============================================================
//  TOUT ÉTEINDRE
// =============================================================
void toutEteindre() {
  digitalWrite(BROCHE_PELTIER_IN1, LOW);
  digitalWrite(BROCHE_PELTIER_IN2, LOW);
  ledcWrite(0, 0);
  digitalWrite(BROCHE_POMPE,     LOW);
  digitalWrite(BROCHE_LED_ROUGE, LOW);
  servo.write(ANGLE_CHARGE);
  pompeActive = false;
}

// =============================================================
//  ERREUR — passage forcé ETAT_ERREUR
// =============================================================
void signalerErreur(const char* message) {
  Serial.printf("[ERREUR] %s\n", message);
  toutEteindre();
  etat = ETAT_ERREUR;
  afficherEcran(ETAT_ERREUR);
}

// =============================================================
//  AFFICHAGE ÉCRAN TFT
// =============================================================
void afficherEcran(int e) {
  ecran.fillScreen(TFT_BLACK);
  ecran.setTextDatum(MC_DATUM);

  switch (e) {

    case ETAT_INIT:
      ecran.fillScreen(TFT_NAVY);
      ecran.setTextColor(TFT_WHITE, TFT_NAVY);
      ecran.setTextSize(2);
      ecran.drawString("PB-0326", 120, 60);
      ecran.setTextSize(1);
      ecran.drawString("Initialisation...", 120, 120);
      ecran.drawString("Purge CO2 en cours", 120, 145);
      break;

    case ETAT_CHECK_CAPTEURS:
      ecran.fillScreen(0x1E9F);
      ecran.setTextColor(TFT_WHITE, 0x1E9F);
      ecran.setTextSize(2);
      ecran.drawString("Detection", 120, 80);
      ecran.setTextSize(1);
      ecran.drawString("Scan I2C...", 120, 140);
      break;

    case ETAT_ATTENTE_DONNEES:
      ecran.fillScreen(0x1E9F);
      ecran.setTextColor(TFT_WHITE, 0x1E9F);
      ecran.setTextSize(2);
      ecran.drawString("Verification", 120, 80);
      ecran.setTextSize(1);
      ecran.drawString("Lecture capteurs...", 120, 140);
      break;

    case ETAT_BUFFER:
      ecran.fillScreen(0xFD20);
      ecran.setTextColor(TFT_BLACK, 0xFD20);
      ecran.setTextSize(2);
      ecran.drawString("PRET", 120, 60);
      ecran.setTextSize(1);
      ecran.drawString("Fermez la valise", 120, 110);
      ecran.drawString("puis appuyez GO", 120, 135);
      break;

    case ETAT_MARCHE: {
      ecran.fillScreen(0x07E0);
      ecran.setTextColor(TFT_BLACK, 0x07E0);
      ecran.setTextSize(2);
      ecran.drawString("EN MARCHE", 120, 30);
      ecran.setTextSize(1);
      char ligne[28];
      snprintf(ligne, sizeof(ligne), "T: %.1f C / %.1f C", temperature, CONSIGNE_TEMP);
      ecran.drawString(ligne, 120, 90);
      snprintf(ligne, sizeof(ligne), "HR: %.1f %%", humidite);
      ecran.drawString(ligne, 120, 120);
      snprintf(ligne, sizeof(ligne), "CO2: %.2f %%", co2);
      ecran.drawString(ligne, 120, 150);
      ecran.drawString(pompeActive ? "Pompe: ON" : "Pompe: OFF", 120, 185);
      break;
    }

    case ETAT_ERREUR:
      ecran.fillScreen(TFT_RED);
      ecran.setTextColor(TFT_WHITE, TFT_RED);
      ecran.setTextSize(2);
      ecran.drawString("ERREUR", 120, 90);
      ecran.setTextSize(1);
      ecran.drawString("Reset requis", 120, 150);
      break;
  }
}
