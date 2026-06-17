// =============================================================
//  PB-0326 — Incubateur transportable pour cellules mammifères
//  HEPIA HES-SO — Groupe 3 : Lyla Founas, Mattéo Pozzo di Borgo
//  Version : 4.0 — Juin 2026
// =============================================================
//
//  MACHINE À ÉTATS v4.0 :
//    0 - INIT    : splash 3s automatique
//    1 - CHECK   : scan I2C toutes les 2s
//                  SHT35 + SCD40 OK → GO → BUFFER
//                  Capteur absent → reste en CHECK, pas d'ERREUR
//                  Rebranché → réinitialisation propre
//    2 - BUFFER  : valeurs live + timer stable + GO → MARCHE
//                  Ventilateur ON dès entrée
//    3 - MARCHE  : régulation active + barres + chrono HH:MM:SS
//                  GO 2s → PAUSE
//    4 - PAUSE   : résistance OFF, ventilateur ON + GO court → MARCHE, GO 3s → BUFFER
//    5 - ERREUR  : tout éteint, cause affichée, LED + buzzer bip, GO 1s → restart
//                  lireCapteurs() bloquée → LED verte reste OFF
//
//  MODIFICATIONS v4.0 (depuis v3.9) :
//    - Peltier remplacé par résistance chauffante (MOSFET GPIO 32)
//      regulerTemperature() simplifiée : 2 états (CHAUFFE / NEUTRE)
//      THERM_DISSIPE supprimé
//    - Ventilateur : MOSFET dédié (GPIO 14), ON dès BUFFER permanent
//      Vérification visuelle en CHECK (ventilON au scan)
//      Pas de feedback électronique (ventilateur 2 fils)
//    - Buzzer passif GPIO 25 : bip digitalWrite synchrone LED rouge en ERREUR
//    - Flag co2ValeurRecue : miseAJourCO2() bloquée tant qu'aucune lecture SCD40 reçue
//    - scannerAdresse() non-bloquant (millis() au lieu de delay)
//    - recupererBusI2C() : Wire.end() + Wire.begin() après erreurs répétées
//    - scd40Lire() découpé : envoi commande / lecture résultat sur deux cycles (2s)
//    - toutEteindre() éteint LED I2C (GPIO 4)
//    - ventilON() mémorisé : digitalWrite émis uniquement au changement d'état
//
//  CORRECTIONS v3.9 maintenues :
//    - sht35.begin() sur objet existant → création nouvel objet
//    - SCD40 rebranché : flag scd40EnChauffe + timer 5s
//    - lireCapteurs() bloquée en ETAT_ERREUR
//    - Barres humidité et CO₂ en MARCHE et PAUSE
//    - ETAT_CHECK : afficherEcran(CHECK) retourne immédiatement
//
// =============================================================

#include <TFT_eSPI.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>

// =============================================================
//  BROCHES GPIO
//  I2C  : SDA=21, SCL=22
//  SPI  : MOSI=23, SCLK=18, CS=15, DC=2, RST=33 (User_Setup.h)
//  EVITER: 6-11 (flash SPI), 34/35/36/39 (input only)
//  EVITER: 13 (HSPI MOSI), 17 (UART2 TX)
// =============================================================
#define BROCHE_RESISTANCE 32
#define BROCHE_VENTILATEUR_CTRL 14
#define BROCHE_SOLENOIDE 27
#define BROCHE_LED_ROUGE 19
#define BROCHE_LED_I2C 4
#define BROCHE_BOUTON_GO 5
#define BROCHE_BUZZER 25

// =============================================================
//  CONSIGNES BIOLOGIQUES
// =============================================================
#define CONSIGNE_TEMP 37.0f
#define SEUIL_CHAUFFE 37.1f
#define SEUIL_REFROIDIT 37.5f
#define TEMP_SECURITE 40.0f
#define CONSIGNE_CO2 3.8f
#define SEUIL_BAS_CO2 3.5f
#define SEUIL_HAUT_CO2 4.0f
#define SEUIL_HR_ALERTE 85.0f

#define BARRE_TEMP_MIN 35.0f
#define BARRE_TEMP_MAX 39.0f
#define BARRE_HR_MIN 0.0f
#define BARRE_HR_MAX 100.0f
#define BARRE_CO2_MIN 0.0f
#define BARRE_CO2_MAX 10.0f

// =============================================================
//  PARAMÈTRES CO₂
// =============================================================
#define DUREE_INJECT_MS 2000
#define DUREE_STABILISATION 10000
#define INTERVALLE_CO2 5000
#define DUREE_CHAUFFE_SCD40 5000

// =============================================================
//  PARAMÈTRES BUZZER
// =============================================================
#define BUZZER_BLINK 500

// =============================================================
//  PARAMÈTRES DIVERS
// =============================================================
#define INTERVALLE_ECRAN 2000
#define INTERVALLE_LECTURE 2000
#define INTERVALLE_SCAN_I2C 2000
#define DUREE_INIT 3000
#define DUREE_ANTIREBOND 50
#define DUREE_APPUI_PAUSE 2000
#define DUREE_APPUI_BUFFER_PAUSE 3000
#define DUREE_APPUI_RESET 1000
#define TENTATIVES_I2C 3
#define DELAI_RETRY_I2C 50
#define SEUIL_ERREURS_LECTURE 3

// =============================================================
//  ÉTATS
// =============================================================
#define ETAT_INIT 0
#define ETAT_CHECK 1
#define ETAT_BUFFER 2
#define ETAT_MARCHE 3
#define ETAT_PAUSE 4
#define ETAT_ERREUR 5

// =============================================================
//  ÉTATS THERMIQUES
// =============================================================
#define THERM_NEUTRE 0
#define THERM_CHAUFFE 1

// =============================================================
//  ÉTATS CO₂
// =============================================================
#define CO2_IDLE 0
#define CO2_INJECTING 1
#define CO2_STABILIZING 2

// =============================================================
//  ADRESSES I2C
// =============================================================
#define ADRESSE_SHT35 0x44
#define ADRESSE_SCD40 0x62

// =============================================================
//  COMMANDES SCD40
// =============================================================
#define SCD40_CMD_START_MEAS 0x21B1
#define SCD40_CMD_READ_MEAS 0xEC05

// =============================================================
//  OBJETS
// =============================================================
TFT_eSPI ecran = TFT_eSPI();
Adafruit_SHT31 sht35 = Adafruit_SHT31();

// =============================================================
//  VARIABLES GLOBALES
// =============================================================

int etat = ETAT_INIT;
int etatPrecedent = -1;
char messageErreur[32] = "";

float temperature = 25.0f;
float humidite = 50.0f;
float co2 = 0.0f;

unsigned long derniereLecture = 0;
unsigned long dernierEcran = 0;
unsigned long tempsInit = 0;
unsigned long dernierScanI2C = 0;
unsigned long tempsDebutMarche = 0;
unsigned long tempsCapteursOK = 0;
unsigned long tempsRedemarrageSCD40 = 0;

int etatCO2 = CO2_IDLE;
unsigned long timerCO2 = 0;
unsigned long dernierCO2 = 0;

bool scd40CommandeEnvoyee = false;

bool sht35Operationnel = false;
bool scd40Demarre = false;
bool scd40EnChauffe = false;
bool co2ValeurRecue = false;
bool alerteHR = false;

bool scanEffectue = false;
bool scanSHT35OK = false;
bool scanSCD40OK = false;

int etatThermiqueActuel = -1;
bool ventilActif = false;  // état mémorisé ventilateur
bool erreurInitialisee = false;
uint8_t compteurErreursSHT35 = 0;
uint8_t compteurErreursSCD40 = 0;

// =============================================================
//  BOUTON
// =============================================================
struct Bouton {
  bool etatPrecedent = false;
  bool appuiDetecte = false;
  unsigned long tempsAppui = 0;
  bool appuiCourtValide = false;
} bouton;

void mettreAJourBouton() {
  bool lu = (digitalRead(BROCHE_BOUTON_GO) == LOW);
  if (lu && !bouton.etatPrecedent) {
    bouton.tempsAppui = millis();
    bouton.appuiDetecte = true;
    bouton.appuiCourtValide = false;
  }
  if (!lu && bouton.etatPrecedent) {
    if (bouton.appuiDetecte && (millis() - bouton.tempsAppui) >= DUREE_ANTIREBOND && (millis() - bouton.tempsAppui) < DUREE_APPUI_PAUSE) {
      bouton.appuiCourtValide = true;
    }
    bouton.appuiDetecte = false;
  }
  bouton.etatPrecedent = lu;
}

bool boutonAppuiCourt() {
  if (bouton.appuiCourtValide) {
    bouton.appuiCourtValide = false;
    return true;
  }
  return false;
}

bool boutonAppuiLong(unsigned long duree) {
  return bouton.etatPrecedent && bouton.appuiDetecte && (millis() - bouton.tempsAppui) >= duree;
}

// =============================================================
//  LED I2C
// =============================================================
void mettreAJourLedI2C() {
  digitalWrite(BROCHE_LED_I2C,
               (sht35Operationnel && scd40Demarre && !scd40EnChauffe) ? HIGH : LOW);
}

// =============================================================
//  VENTILATEUR — état mémorisé
//  digitalWrite émis UNIQUEMENT au changement d'état
// =============================================================
void ventilON() {
  if (ventilActif) return;
  ventilActif = true;
  digitalWrite(BROCHE_VENTILATEUR_CTRL, HIGH);
}

void ventilOFF() {
  if (!ventilActif) return;
  ventilActif = false;
  digitalWrite(BROCHE_VENTILATEUR_CTRL, LOW);
}

// =============================================================
//  TOUT ÉTEINDRE
// =============================================================
void toutEteindre() {
  digitalWrite(BROCHE_RESISTANCE, LOW);
  ventilOFF();
  digitalWrite(BROCHE_SOLENOIDE, LOW);
  digitalWrite(BROCHE_LED_ROUGE, LOW);
  digitalWrite(BROCHE_LED_I2C, LOW);
  digitalWrite(BROCHE_BUZZER, LOW);
  etatThermiqueActuel = -1;
}

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("=== PB-0326 Firmware v4.0 ===");

  pinMode(BROCHE_RESISTANCE, OUTPUT);
  pinMode(BROCHE_VENTILATEUR_CTRL, OUTPUT);
  pinMode(BROCHE_SOLENOIDE, OUTPUT);
  pinMode(BROCHE_LED_ROUGE, OUTPUT);
  pinMode(BROCHE_LED_I2C, OUTPUT);
  pinMode(BROCHE_BOUTON_GO, INPUT_PULLUP);
  pinMode(BROCHE_BUZZER, OUTPUT);

  toutEteindre();

  Wire.begin(21, 22);
  delay(100);

  if (sht35.begin(ADRESSE_SHT35)) {
    sht35Operationnel = true;
    Serial.println("[SHT35] OK");
  } else {
    Serial.println("[SHT35] ABSENT au démarrage");
  }

  scd40Demarrer();
  mettreAJourLedI2C();

  ecran.init();
  ecran.setRotation(0);
  ecran.fillScreen(TFT_BLACK);
  ecran.setTextDatum(MC_DATUM);
  ecran.fillScreen(TFT_WHITE);
  delay(500);
  ecran.fillScreen(TFT_BLACK);

  tempsInit = millis();
  Serial.println("[SETUP] OK");
}

// =============================================================
//  LOOP
// =============================================================
void loop() {

  mettreAJourBouton();

  if (scd40EnChauffe && (millis() - tempsRedemarrageSCD40 >= DUREE_CHAUFFE_SCD40)) {
    scd40EnChauffe = false;
    Serial.println("[SCD40] Chauffe terminée");
    mettreAJourLedI2C();
  }

  if (etat != ETAT_ERREUR && millis() - derniereLecture >= INTERVALLE_LECTURE) {
    lireCapteurs();
    derniereLecture = millis();
  }

  if (etat != etatPrecedent) {
    afficherEcran(etat);
    etatPrecedent = etat;
    if (etat != ETAT_CHECK) scanEffectue = false;
    if (etat != ETAT_ERREUR) erreurInitialisee = false;
    Serial.printf("[ÉTAT] → %d\n", etat);
  }

  switch (etat) {

    case ETAT_INIT:
      if (millis() - tempsInit >= DUREE_INIT) etat = ETAT_CHECK;
      break;

    case ETAT_CHECK:
      if (!scanEffectue || (millis() - dernierScanI2C >= INTERVALLE_SCAN_I2C)) {
        scannerCapteursI2C();
        dernierScanI2C = millis();
        scanEffectue = true;
        if (!scanSHT35OK || !scanSCD40OK || scd40EnChauffe) {
          tempsCapteursOK = 0;
        } else if (tempsCapteursOK == 0) {
          tempsCapteursOK = millis();
        }
      }
      if (boutonAppuiCourt()) {
        if (scanSHT35OK && scanSCD40OK && !scd40EnChauffe) {
          etat = ETAT_BUFFER;
        } else {
          Serial.println("[CHECK] GO ignoré — système non prêt");
        }
      }
      break;

    case ETAT_BUFFER:
      ventilON();
      if (millis() - dernierEcran >= INTERVALLE_ECRAN) {
        afficherEcran(ETAT_BUFFER);
        dernierEcran = millis();
      }
      if (boutonAppuiCourt()) {
        Serial.println("[BUFFER] GO → MARCHE");
        tempsDebutMarche = millis();
        dernierCO2 = millis();
        dernierEcran = millis();
        etatCO2 = CO2_IDLE;
        co2ValeurRecue = false;
        scd40CommandeEnvoyee = false;
        compteurErreursSHT35 = 0;
        compteurErreursSCD40 = 0;
        etatThermiqueActuel = -1;
        etat = ETAT_MARCHE;
      }
      break;

    case ETAT_MARCHE:
      if (temperature >= TEMP_SECURITE) {
        signalerErreur("TEMP CRITIQUE");
        break;
      }
      ventilON();
      regulerTemperature();
      miseAJourCO2();
      if (millis() - dernierEcran >= INTERVALLE_ECRAN) {
        afficherEcran(ETAT_MARCHE);
        dernierEcran = millis();
      }
      if (boutonAppuiLong(DUREE_APPUI_PAUSE)) {
        Serial.println("[MARCHE] GO 2s → PAUSE");
        etat = ETAT_PAUSE;
      }
      break;

    case ETAT_PAUSE:
      if (etatThermiqueActuel != THERM_NEUTRE) {
        digitalWrite(BROCHE_RESISTANCE, LOW);
        etatThermiqueActuel = THERM_NEUTRE;
        Serial.println("[PAUSE] Résistance OFF");
      }
      ventilON();
      if (millis() - dernierEcran >= INTERVALLE_ECRAN) {
        afficherEcran(ETAT_PAUSE);
        dernierEcran = millis();
      }
      if (boutonAppuiLong(DUREE_APPUI_BUFFER_PAUSE)) {
        Serial.println("[PAUSE] GO 3s → BUFFER");
        bouton.appuiCourtValide = false;
        etatThermiqueActuel = -1;
        compteurErreursSHT35 = 0;
        compteurErreursSCD40 = 0;
        dernierEcran = millis();
        etat = ETAT_BUFFER;
      } else if (boutonAppuiCourt()) {
        Serial.println("[PAUSE] GO → MARCHE");
        etatThermiqueActuel = -1;
        compteurErreursSHT35 = 0;
        compteurErreursSCD40 = 0;
        dernierEcran = millis();
        etat = ETAT_MARCHE;
      }
      break;

    case ETAT_ERREUR:
      if (!erreurInitialisee) {
        toutEteindre();
        erreurInitialisee = true;
        Serial.println("[ERREUR] Actionneurs OFF");
      }
      {
        static unsigned long dernierBlink = 0;
        static bool ledActive = false;
        if (millis() - dernierBlink >= BUZZER_BLINK) {
          ledActive = !ledActive;
          digitalWrite(BROCHE_LED_ROUGE, ledActive ? HIGH : LOW);
          digitalWrite(BROCHE_BUZZER, ledActive ? HIGH : LOW);
          dernierBlink = millis();
        }
      }
      if (boutonAppuiLong(DUREE_APPUI_RESET)) {
        Serial.println("[ERREUR] → restart");
        delay(200);
        ESP.restart();
      }
      break;
  }
}

// =============================================================
//  SCD40 — Démarrage
// =============================================================
void scd40Demarrer() {
  Wire.beginTransmission(ADRESSE_SCD40);
  Wire.write((uint8_t)(SCD40_CMD_START_MEAS >> 8));
  Wire.write((uint8_t)(SCD40_CMD_START_MEAS & 0xFF));
  uint8_t err = Wire.endTransmission();
  if (err == 0) {
    scd40Demarre = true;
    scd40EnChauffe = true;
    tempsRedemarrageSCD40 = millis();
    scd40CommandeEnvoyee = false;
    Serial.println("[SCD40] Démarré — chauffe 5s");
  } else {
    scd40Demarre = false;
    scd40EnChauffe = false;
    Serial.printf("[SCD40] ABSENT (err=%d)\n", err);
  }
}

// =============================================================
//  SCD40 — CRC-8
// =============================================================
uint8_t calculerCRC(uint8_t* data, uint8_t len) {
  uint8_t crc = 0xFF;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
    }
  }
  return crc;
}

// =============================================================
//  SCD40 — Envoi commande READ (cycle N)
// =============================================================
void scd40EnvoyerCommande() {
  Wire.beginTransmission(ADRESSE_SCD40);
  Wire.write((uint8_t)(SCD40_CMD_READ_MEAS >> 8));
  Wire.write((uint8_t)(SCD40_CMD_READ_MEAS & 0xFF));
  if (Wire.endTransmission() == 0) {
    scd40CommandeEnvoyee = true;
    Serial.println("[SCD40] Commande READ envoyée");
  } else {
    scd40CommandeEnvoyee = false;
    Serial.println("[SCD40] Echec envoi commande READ");
  }
}

// =============================================================
//  SCD40 — Lecture résultat (cycle N+1)
// =============================================================
bool scd40LireResultat(float& co2_out, float& temp_out, float& hum_out) {
  Wire.requestFrom((uint8_t)ADRESSE_SCD40, (uint8_t)9);
  if (Wire.available() < 9) return false;

  uint8_t data[9];
  for (uint8_t i = 0; i < 9; i++) data[i] = Wire.read();

  if (calculerCRC(data, 2) != data[2]) return false;
  if (calculerCRC(data + 3, 2) != data[5]) return false;
  if (calculerCRC(data + 6, 2) != data[8]) return false;

  uint16_t raw_co2 = ((uint16_t)data[0] << 8) | data[1];
  uint16_t raw_temp = ((uint16_t)data[3] << 8) | data[4];
  uint16_t raw_hum = ((uint16_t)data[6] << 8) | data[7];

  co2_out = (float)raw_co2;
  temp_out = -45.0f + 175.0f * (raw_temp / 65535.0f);
  hum_out = 100.0f * (raw_hum / 65535.0f);
  return true;
}

// =============================================================
//  RÉCUPÉRATION BUS I2C
// =============================================================
void recupererBusI2C() {
  Serial.println("[I2C] Recovery bus...");
  Wire.end();
  delay(10);
  Wire.begin(21, 22);
  delay(10);
  Serial.println("[I2C] Bus réinitialisé");
}

// =============================================================
//  LECTURE CAPTEURS
// =============================================================
void lireCapteurs() {

  if (sht35Operationnel) {
    float t = sht35.readTemperature();
    float h = sht35.readHumidity();
    if (!isnan(t) && !isnan(h)) {
      temperature = t;
      humidite = h;
      compteurErreursSHT35 = 0;
      if (humidite < SEUIL_HR_ALERTE) {
        if (!alerteHR) {
          alerteHR = true;
          Serial.printf("[HR] ALERTE HR=%.1f%%\n", humidite);
        }
      } else if (alerteHR) {
        alerteHR = false;
        Serial.printf("[HR] Alerte levée HR=%.1f%%\n", humidite);
      }
      Serial.printf("[SHT35] T=%.1fC  HR=%.1f%%\n", temperature, humidite);
    } else {
      compteurErreursSHT35++;
      Serial.printf("[SHT35] NaN (%d/%d)\n", compteurErreursSHT35, SEUIL_ERREURS_LECTURE);
      if (compteurErreursSHT35 >= SEUIL_ERREURS_LECTURE) {
        recupererBusI2C();
        if (etat == ETAT_MARCHE) {
          signalerErreur("SHT35 perdu");
          return;
        }
      }
    }
  }

  if (scd40Demarre && !scd40EnChauffe) {
    if (!scd40CommandeEnvoyee) {
      scd40EnvoyerCommande();
    } else {
      float co2_ppm, temp_scd, hum_scd;
      if (scd40LireResultat(co2_ppm, temp_scd, hum_scd)) {
        co2 = co2_ppm / 10000.0f;
        co2ValeurRecue = true;
        compteurErreursSCD40 = 0;
        scd40CommandeEnvoyee = false;
        Serial.printf("[SCD40] CO2=%.2f%%\n", co2);
      } else {
        compteurErreursSCD40++;
        scd40CommandeEnvoyee = false;
        Serial.printf("[SCD40] Erreur (%d/%d)\n", compteurErreursSCD40, SEUIL_ERREURS_LECTURE);
        if (compteurErreursSCD40 >= SEUIL_ERREURS_LECTURE) {
          recupererBusI2C();
          if (etat == ETAT_MARCHE) {
            signalerErreur("SCD40 perdu");
            return;
          }
        }
      }
    }
  }

  mettreAJourLedI2C();
}

// =============================================================
//  SCAN I2C — non-bloquant
// =============================================================
bool scannerAdresse(uint8_t adresse, uint8_t tentatives) {
  for (uint8_t i = 0; i < tentatives; i++) {
    Wire.beginTransmission(adresse);
    if (Wire.endTransmission() == 0) return true;
    if (i < tentatives - 1) {
      unsigned long t = millis();
      while (millis() - t < DELAI_RETRY_I2C) {}
    }
  }
  return false;
}

// =============================================================
//  SCAN CAPTEURS I2C + VENTILATEUR
// =============================================================
void scannerCapteursI2C() {
  Serial.println("[I2C] Scan...");

  scanSHT35OK = scannerAdresse(ADRESSE_SHT35, TENTATIVES_I2C);
  if (scanSHT35OK && !sht35Operationnel) {
    sht35 = Adafruit_SHT31();
    sht35Operationnel = sht35.begin(ADRESSE_SHT35);
    if (sht35Operationnel) Serial.println("[SHT35] Réinitialisé OK");
    else {
      Serial.println("[SHT35] Réinit échouée");
      scanSHT35OK = false;
    }
  } else if (!scanSHT35OK) {
    sht35Operationnel = false;
  }
  Serial.printf("[I2C] SHT35 → %s\n", scanSHT35OK ? "OK" : "ABSENT");

  scanSCD40OK = scannerAdresse(ADRESSE_SCD40, TENTATIVES_I2C);
  if (scanSCD40OK && !scd40Demarre) scd40Demarrer();
  else if (!scanSCD40OK) {
    scd40Demarre = false;
    scd40EnChauffe = false;
  }
  Serial.printf("[I2C] SCD40 → %s%s\n",
                scanSCD40OK ? "OK" : "ABSENT",
                scd40EnChauffe ? " (chauffe)" : "");

  ventilON();
  Serial.println("[VENTIL] ON");

  mettreAJourLedI2C();
  afficherResultatI2C(scanSHT35OK, scanSCD40OK);
}

// =============================================================
//  RÉGULATION THERMIQUE
// =============================================================
void regulerTemperature() {
  int nouvelEtat = (temperature < SEUIL_CHAUFFE) ? THERM_CHAUFFE : THERM_NEUTRE;
  if (nouvelEtat == etatThermiqueActuel) return;
  etatThermiqueActuel = nouvelEtat;
  switch (nouvelEtat) {
    case THERM_CHAUFFE:
      digitalWrite(BROCHE_RESISTANCE, HIGH);
      Serial.printf("[THERM] Resistance ON  T=%.1fC\n", temperature);
      break;
    case THERM_NEUTRE:
      digitalWrite(BROCHE_RESISTANCE, LOW);
      Serial.printf("[THERM] Resistance OFF T=%.1fC\n", temperature);
      break;
  }
}

// =============================================================
//  RÉGULATION CO₂
// =============================================================
void miseAJourCO2() {
  if (!co2ValeurRecue) return;
  unsigned long maintenant = millis();
  switch (etatCO2) {
    case CO2_IDLE:
      if (maintenant - dernierCO2 >= INTERVALLE_CO2) {
        dernierCO2 = maintenant;
        if (co2 < SEUIL_BAS_CO2) {
          digitalWrite(BROCHE_SOLENOIDE, HIGH);
          timerCO2 = maintenant;
          etatCO2 = CO2_INJECTING;
          Serial.printf("[CO2] Injection CO2=%.2f%%\n", co2);
        } else if (co2 > SEUIL_HAUT_CO2) {
          Serial.printf("[CO2] Trop élevé CO2=%.2f%%\n", co2);
        }
      }
      break;
    case CO2_INJECTING:
      if (maintenant - timerCO2 >= DUREE_INJECT_MS) {
        digitalWrite(BROCHE_SOLENOIDE, LOW);
        timerCO2 = maintenant;
        etatCO2 = CO2_STABILIZING;
        Serial.println("[CO2] Vanne fermée");
      }
      break;
    case CO2_STABILIZING:
      if (maintenant - timerCO2 >= DUREE_STABILISATION) {
        etatCO2 = CO2_IDLE;
        Serial.println("[CO2] Stabilisation terminée");
      }
      break;
  }
}

// =============================================================
//  SIGNALER ERREUR
// =============================================================
void signalerErreur(const char* message) {
  Serial.printf("[ERREUR] %s\n", message);
  strncpy(messageErreur, message, sizeof(messageErreur) - 1);
  toutEteindre();
  compteurErreursSHT35 = 0;
  compteurErreursSCD40 = 0;
  erreurInitialisee = false;
  etat = ETAT_ERREUR;
  afficherEcran(ETAT_ERREUR);
}

// =============================================================
//  BARRE DE PROGRESSION
// =============================================================
void dessinBarre(int x, int y, int w, int h,
                 float valeur, float valMin, float valMax,
                 float zoneOKmin, float zoneOKmax, float consigne) {
  ecran.fillRect(x, y, w, h, 0x4208);
  float ratio = constrain((valeur - valMin) / (valMax - valMin), 0.0f, 1.0f);
  int largeur = (int)(ratio * w);
  uint16_t couleur;
  if (valeur >= zoneOKmin && valeur <= zoneOKmax) couleur = TFT_GREEN;
  else if (valeur < zoneOKmin - (zoneOKmax - zoneOKmin) * 0.2f || valeur > zoneOKmax + (zoneOKmax - zoneOKmin) * 0.2f) couleur = 0xF800;
  else couleur = 0xFD20;
  ecran.fillRect(x, y, largeur, h, couleur);
  float ratioOKmin = constrain((zoneOKmin - valMin) / (valMax - valMin), 0.0f, 1.0f);
  ecran.drawFastVLine(x + (int)(ratioOKmin * w), y - 2, h + 4, TFT_WHITE);
  float ratioOKmax = constrain((zoneOKmax - valMin) / (valMax - valMin), 0.0f, 1.0f);
  ecran.drawFastVLine(x + (int)(ratioOKmax * w), y - 2, h + 4, TFT_WHITE);
  if (consigne >= 0) {
    float ratioC = constrain((consigne - valMin) / (valMax - valMin), 0.0f, 1.0f);
    ecran.drawFastVLine(x + (int)(ratioC * w), y - 4, h + 8, TFT_YELLOW);
  }
  ecran.drawRect(x, y, w, h, TFT_WHITE);
}

// =============================================================
//  AFFICHAGE RÉSULTAT I2C
// =============================================================
void afficherResultatI2C(bool sht35_ok, bool scd40_ok) {
  ecran.fillScreen(0x1E9F);
  ecran.setTextDatum(MC_DATUM);
  ecran.setTextColor(TFT_WHITE, 0x1E9F);
  ecran.setTextSize(2);
  ecran.drawString("Verification", 120, 20);
  ecran.setTextSize(1);

  ecran.setTextColor(sht35_ok ? TFT_GREEN : TFT_RED, 0x1E9F);
  ecran.drawString(sht35_ok ? "SHT35 [0x44]  OK" : "SHT35 [0x44]  ABSENT", 120, 80);

  if (scd40_ok && scd40EnChauffe) {
    ecran.setTextColor(TFT_YELLOW, 0x1E9F);
    ecran.drawString("SCD40 [0x62]  CHAUFFE...", 120, 110);
  } else {
    ecran.setTextColor(scd40_ok ? TFT_GREEN : TFT_RED, 0x1E9F);
    ecran.drawString(scd40_ok ? "SCD40 [0x62]  OK" : "SCD40 [0x62]  ABSENT", 120, 110);
  }

  bool tousOK = sht35_ok && scd40_ok && !scd40EnChauffe;
  ecran.setTextSize(2);
  ecran.setTextColor(tousOK ? TFT_GREEN : (sht35_ok || scd40_ok) ? TFT_YELLOW
                                                                 : TFT_RED,
                     0x1E9F);
  ecran.drawString(tousOK ? "SYSTEME OK" : scd40EnChauffe ? "ATTENTE SCD40"
                                                          : "ATTENTE...",
                   120, 155);

  ecran.setTextSize(1);
  ecran.setTextColor(TFT_WHITE, 0x1E9F);
  if (tousOK && tempsCapteursOK > 0) {
    char buf[32];
    snprintf(buf, sizeof(buf), "Stable depuis %lus", (millis() - tempsCapteursOK) / 1000);
    ecran.drawString(buf, 120, 200);
    ecran.drawString("GO pour continuer", 120, 220);
  } else if (scd40EnChauffe) {
    char buf[32];
    uint32_t restant = DUREE_CHAUFFE_SCD40 / 1000 - min((millis() - tempsRedemarrageSCD40) / 1000UL, (unsigned long)DUREE_CHAUFFE_SCD40 / 1000);
    snprintf(buf, sizeof(buf), "SCD40 pret dans %lus", restant);
    ecran.drawString(buf, 120, 200);
  } else {
    ecran.drawString("Brancher SHT35 + SCD40", 120, 200);
    ecran.drawString("GO ignore si capteur absent", 120, 220);
  }
}

// =============================================================
//  AFFICHAGE ÉCRAN TFT
// =============================================================
void afficherEcran(int e) {
  ecran.setTextDatum(MC_DATUM);
  switch (e) {

    case ETAT_INIT:
      ecran.fillScreen(TFT_NAVY);
      ecran.setTextColor(TFT_WHITE, TFT_NAVY);
      ecran.setTextSize(2);
      ecran.drawString("PB-0326", 120, 60);
      ecran.setTextSize(1);
      ecran.drawString("Incubateur transportable", 120, 110);
      ecran.drawString("HEPIA HES-SO  Groupe 3", 120, 135);
      ecran.drawString("v4.0", 120, 160);
      ecran.drawString("Initialisation...", 120, 200);
      break;

    case ETAT_CHECK:
      return;

    case ETAT_BUFFER:
      {
        ecran.fillScreen(0xFD20);
        ecran.setTextColor(TFT_BLACK, 0xFD20);
        ecran.setTextSize(2);
        ecran.drawString("PRET", 120, 22);
        ecran.setTextSize(1);
        char buf[32];
        snprintf(buf, sizeof(buf), "T:   %.1f C", temperature);
        ecran.drawString(buf, 120, 65);
        snprintf(buf, sizeof(buf), "HR:  %.1f %%", humidite);
        ecran.drawString(buf, 120, 85);
        snprintf(buf, sizeof(buf), "CO2: %.2f %%", co2);
        ecran.drawString(buf, 120, 105);
        ecran.setTextColor(TFT_CYAN, 0xFD20);
        ecran.drawString("Ventilateur ON", 120, 128);
        if (tempsCapteursOK > 0) {
          snprintf(buf, sizeof(buf), "Stable depuis %lus", (millis() - tempsCapteursOK) / 1000);
          ecran.setTextColor(TFT_GREEN, 0xFD20);
          ecran.drawString(buf, 120, 150);
        }
        ecran.setTextColor(TFT_BLACK, 0xFD20);
        ecran.drawString("Fermez l'enceinte", 120, 185);
        ecran.drawString("GO pour demarrer", 120, 205);
        break;
      }

    case ETAT_MARCHE:
      {
        ecran.fillScreen(0x07E0);
        ecran.setTextColor(TFT_BLACK, 0x07E0);
        ecran.setTextSize(2);
        ecran.drawString("EN MARCHE", 120, 15);
        ecran.setTextSize(1);
        char ligne[32];
        unsigned long elapsed = (millis() - tempsDebutMarche) / 1000;
        snprintf(ligne, sizeof(ligne), "%02d:%02d:%02d",
                 (uint8_t)(elapsed / 3600),
                 (uint8_t)((elapsed % 3600) / 60),
                 (uint8_t)(elapsed % 60));
        ecran.drawString(ligne, 120, 35);

        snprintf(ligne, sizeof(ligne), "T: %.1f/%.1f C", temperature, CONSIGNE_TEMP);
        ecran.drawString(ligne, 120, 58);
        dessinBarre(20, 68, 200, 10, temperature, BARRE_TEMP_MIN, BARRE_TEMP_MAX, SEUIL_CHAUFFE, SEUIL_REFROIDIT, CONSIGNE_TEMP);
        ecran.setTextColor(TFT_BLACK, 0x07E0);
        ecran.drawString("35", 20, 82);
        ecran.drawString("37", 113, 82);
        ecran.drawString("39", 212, 82);

        if (alerteHR) {
          ecran.setTextColor(TFT_RED, 0x07E0);
          snprintf(ligne, sizeof(ligne), "HR: %.1f %% ALERTE", humidite);
        } else {
          snprintf(ligne, sizeof(ligne), "HR: %.1f %%", humidite);
        }
        ecran.drawString(ligne, 120, 100);
        dessinBarre(20, 110, 200, 10, humidite, BARRE_HR_MIN, BARRE_HR_MAX, SEUIL_HR_ALERTE, 100.0f, 95.0f);
        ecran.setTextColor(TFT_BLACK, 0x07E0);
        ecran.drawString("0", 20, 124);
        ecran.drawString("50", 107, 124);
        ecran.drawString("100", 205, 124);

        const char* indCO2 = (etatCO2 == CO2_INJECTING) ? " [INJ]" : (etatCO2 == CO2_STABILIZING) ? " [STB]"
                                                                                                  : "";
        snprintf(ligne, sizeof(ligne), "CO2: %.2f%%%s", co2, indCO2);
        ecran.drawString(ligne, 120, 142);
        dessinBarre(20, 152, 200, 10, co2, BARRE_CO2_MIN, BARRE_CO2_MAX, SEUIL_BAS_CO2, SEUIL_HAUT_CO2, CONSIGNE_CO2);
        ecran.setTextColor(TFT_BLACK, 0x07E0);
        ecran.drawString("0", 20, 166);
        ecran.drawString("5%", 110, 166);
        ecran.drawString("10%", 203, 166);

        ecran.drawString((etatThermiqueActuel == THERM_CHAUFFE) ? "Resistance ON" : "Resistance OFF", 120, 185);

        if (compteurErreursSHT35 > 0 || compteurErreursSCD40 > 0) {
          ecran.setTextColor(TFT_YELLOW, 0x07E0);
          snprintf(ligne, sizeof(ligne), "Err T:%d CO2:%d", compteurErreursSHT35, compteurErreursSCD40);
          ecran.drawString(ligne, 120, 202);
          ecran.setTextColor(TFT_BLACK, 0x07E0);
        }
        ecran.drawString("GO 2s → pause", 120, 258);
        break;
      }

    case ETAT_PAUSE:
      {
        ecran.fillScreen(0xFC60);
        ecran.setTextColor(TFT_BLACK, 0xFC60);
        ecran.setTextSize(2);
        ecran.drawString("EN PAUSE", 120, 15);
        ecran.setTextSize(1);
        char buf[32];
        if (tempsDebutMarche > 0) {
          unsigned long elapsed = (millis() - tempsDebutMarche) / 1000;
          snprintf(buf, sizeof(buf), "%02d:%02d:%02d",
                   (uint8_t)(elapsed / 3600),
                   (uint8_t)((elapsed % 3600) / 60),
                   (uint8_t)(elapsed % 60));
          ecran.drawString(buf, 120, 35);
        }
        ecran.drawString("Resistance OFF", 120, 55);
        ecran.setTextColor(TFT_CYAN, 0xFC60);
        ecran.drawString("Ventilateur ON", 120, 68);
        ecran.setTextColor(TFT_BLACK, 0xFC60);

        snprintf(buf, sizeof(buf), "T: %.1f C", temperature);
        ecran.drawString(buf, 120, 88);
        dessinBarre(20, 98, 200, 8, temperature, BARRE_TEMP_MIN, BARRE_TEMP_MAX, SEUIL_CHAUFFE, SEUIL_REFROIDIT, CONSIGNE_TEMP);
        ecran.setTextColor(TFT_BLACK, 0xFC60);

        snprintf(buf, sizeof(buf), "HR: %.1f %%", humidite);
        ecran.drawString(buf, 120, 118);
        dessinBarre(20, 128, 200, 8, humidite, BARRE_HR_MIN, BARRE_HR_MAX, SEUIL_HR_ALERTE, 100.0f, 95.0f);
        ecran.setTextColor(TFT_BLACK, 0xFC60);

        snprintf(buf, sizeof(buf), "CO2: %.2f %%", co2);
        ecran.drawString(buf, 120, 148);
        dessinBarre(20, 158, 200, 8, co2, BARRE_CO2_MIN, BARRE_CO2_MAX, SEUIL_BAS_CO2, SEUIL_HAUT_CO2, CONSIGNE_CO2);
        ecran.setTextColor(TFT_BLACK, 0xFC60);

        ecran.drawString("GO court  → reprendre", 120, 220);
        ecran.drawString("GO 3s     → retour PRET", 120, 240);
        break;
      }

    case ETAT_ERREUR:
      ecran.fillScreen(TFT_RED);
      ecran.setTextColor(TFT_WHITE, TFT_RED);
      ecran.setTextSize(2);
      ecran.drawString("ERREUR", 120, 60);
      ecran.setTextSize(1);
      ecran.drawString(messageErreur, 120, 110);
      ecran.drawString("Systeme arrete", 120, 135);
      ecran.drawString("Actionneurs OFF", 120, 155);
      ecran.drawString("GO 1s → redemarrer", 120, 205);
      break;
  }
}