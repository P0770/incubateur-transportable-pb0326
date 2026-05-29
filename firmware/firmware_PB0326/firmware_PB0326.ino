// =============================================================
//  PB-0326 — Incubateur transportable pour cellules mammifères
//  HEPIA HES-SO — Groupe 3 : Lyla Founas, Mattéo Pozzo di Borgo
//  Version : 3.8 — Mai 2026
// =============================================================
//
//  MACHINE À ÉTATS v3.8 :
//    0 - INIT    : splash 3s automatique
//    1 - CHECK   : scan I2C toutes les 2s, attente GO
//                  SHT35 + SCD40 OK → GO → BUFFER
//                  Capteur absent → reste en CHECK, pas d'ERREUR
//    2 - BUFFER  : valeurs live + timer "stable depuis Xs" + GO
//    3 - MARCHE  : régulation active, chrono transport, GO 2s → PAUSE
//    4 - PAUSE   : actionneurs OFF, GO court → MARCHE, GO 3s → BUFFER
//    5 - ERREUR  : tout éteint, message cause affiché, GO 1s → restart
//
//  MODIFICATIONS v3.8 :
//    - ETAT_ATTENTE_DONNEES supprimé
//    - ETAT_CHECK_CAPTEURS : perte capteur reste en CHECK (pas ERREUR)
//    - SCD40 réactivé : scan I2C, lecture toutes les 2s, perte → ERREUR en MARCHE
//    - LED I2C ON seulement si SHT35 ET SCD40 détectés
//    - Message cause erreur affiché sur écran ERREUR
//    - Barre de progression température en MARCHE
//    - Chrono transport HH:MM:SS en MARCHE
//    - Timer "stable depuis Xs" en BUFFER
//    - miseAJourHumidite() intégrée dans lireCapteurs()
//    - scd40EnvoyerCommande() supprimée, appels directs inline
//    - compteurErreursSCD40 : même logique que SHT35
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
//  EVITER: 13 (HSPI MOSI), 25/26 (DAC1/DAC2)
// =============================================================
#define BROCHE_PELTIER 32
#define BROCHE_VENTILATEUR 14
#define BROCHE_SOLENOIDE 27
#define BROCHE_LED_ROUGE 19
#define BROCHE_LED_I2C 4
#define BROCHE_BOUTON_GO 5

// =============================================================
//  CONSIGNES BIOLOGIQUES
// =============================================================
#define CONSIGNE_TEMP 37.0f
#define SEUIL_CHAUFFE 36.5f
#define SEUIL_REFROIDIT 37.5f
#define TEMP_SECURITE 40.0f
#define CONSIGNE_CO2 5.0f
#define SEUIL_BAS_CO2 4.7f
#define SEUIL_HAUT_CO2 5.3f
#define SEUIL_HR_ALERTE 85.0f

// Plage affichage barre température (°C)
#define BARRE_TEMP_MIN 35.0f
#define BARRE_TEMP_MAX 39.0f

// =============================================================
//  PARAMÈTRES CO₂
// =============================================================
#define DUREE_INJECT_MS 200
#define DUREE_STABILISATION 30000
#define INTERVALLE_CO2 5000

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
#define SEUIL_ERREURS_LECTURE 3  // NaN consécutifs → ERREUR en MARCHE

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
#define THERM_DISSIPE 2

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
#define SCD40_CMD_STOP_MEAS 0x3F86

// =============================================================
//  OBJETS
// =============================================================
TFT_eSPI ecran = TFT_eSPI();
Adafruit_SHT31 sht35 = Adafruit_SHT31();

// =============================================================
//  VARIABLES GLOBALES
// =============================================================

// --- Machine à états ---
int etat = ETAT_INIT;
int etatPrecedent = -1;
char messageErreur[32] = "";  // Message cause erreur affiché sur écran

// --- Valeurs capteurs ---
float temperature = 25.0f;
float humidite = 50.0f;
float co2 = 0.0f;  // ppm / 10000 → %

// --- Timers ---
unsigned long derniereLecture = 0;
unsigned long dernierEcran = 0;
unsigned long tempsInit = 0;
unsigned long dernierScanI2C = 0;
unsigned long tempsDebutMarche = 0;  // Horodatage début transport
unsigned long tempsCapteursOK = 0;   // Horodatage depuis capteurs tous OK

// --- CO₂ ---
int etatCO2 = CO2_IDLE;
unsigned long timerCO2 = 0;
unsigned long dernierCO2 = 0;

// --- Flags capteurs ---
bool sht35Operationnel = false;
bool scd40Demarre = false;
bool alerteHR = false;

// --- Scan I2C ---
bool scanEffectue = false;
bool scanSHT35OK = false;
bool scanSCD40OK = false;

// --- État thermique mémorisé ---
int etatThermiqueActuel = -1;

// --- Erreur one-shot ---
bool erreurInitialisee = false;

// --- Compteurs perte capteur en MARCHE ---
uint8_t compteurErreursSHT35 = 0;
uint8_t compteurErreursSCD40 = 0;

// =============================================================
//  STRUCTURE BOUTON — anti-rebond non bloquant
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
//  LED I2C — ON si SHT35 ET SCD40 opérationnels
// =============================================================
void mettreAJourLedI2C() {
  digitalWrite(BROCHE_LED_I2C,
               (sht35Operationnel && scd40Demarre) ? HIGH : LOW);
}

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("=== PB-0326 Firmware v3.8 ===");

  pinMode(BROCHE_PELTIER, OUTPUT);
  pinMode(BROCHE_VENTILATEUR, OUTPUT);
  pinMode(BROCHE_SOLENOIDE, OUTPUT);
  pinMode(BROCHE_LED_ROUGE, OUTPUT);
  pinMode(BROCHE_LED_I2C, OUTPUT);
  pinMode(BROCHE_BOUTON_GO, INPUT_PULLUP);

  toutEteindre();

  Wire.begin(21, 22);
  delay(100);

  // Tentative init SHT35 au démarrage
  if (sht35.begin(ADRESSE_SHT35)) {
    sht35Operationnel = true;
    Serial.println("[SHT35] OK");
  } else {
    Serial.println("[SHT35] ABSENT au démarrage");
  }

  // Tentative init SCD40 au démarrage
  scd40Demarrer();

  mettreAJourLedI2C();

  // Écran TFT — broches dans User_Setup.h
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

  // Lecture capteurs toutes les 2s
  if (millis() - derniereLecture >= INTERVALLE_LECTURE) {
    lireCapteurs();
    derniereLecture = millis();
  }

  // Refresh TFT au changement d'état
  if (etat != etatPrecedent) {
    afficherEcran(etat);
    etatPrecedent = etat;
    if (etat != ETAT_CHECK) scanEffectue = false;
    if (etat != ETAT_ERREUR) erreurInitialisee = false;
    Serial.printf("[ÉTAT] → %d\n", etat);
  }

  switch (etat) {

    // ----------------------------------------------------------
    //  ÉTAT 0 — INIT
    // ----------------------------------------------------------
    case ETAT_INIT:
      if (millis() - tempsInit >= DUREE_INIT) {
        etat = ETAT_CHECK;
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 1 — CHECK
    //
    //  Scan I2C toutes les 2s.
    //  SHT35 + SCD40 tous les deux OK + GO → BUFFER
    //  Capteur absent → reste en CHECK, affiche ABSENT, pas d'ERREUR
    //  Timer tempsCapteursOK remis à 0 tant qu'un capteur manque
    // ----------------------------------------------------------
    case ETAT_CHECK:
      if (!scanEffectue || (millis() - dernierScanI2C >= INTERVALLE_SCAN_I2C)) {
        scannerCapteursI2C();
        dernierScanI2C = millis();
        scanEffectue = true;

        // Reset timer stabilité si un capteur manque
        if (!scanSHT35OK || !scanSCD40OK) {
          tempsCapteursOK = 0;
        } else if (tempsCapteursOK == 0) {
          tempsCapteursOK = millis();  // Démarrer timer quand les deux sont OK
        }
      }


      // GO → BUFFER si les deux capteurs OK
      if (boutonAppuiCourt()) {
        if (scanSHT35OK && scanSCD40OK) {
          etat = ETAT_BUFFER;
        } else {
          Serial.println("[CHECK] GO ignoré — capteurs manquants");
        }
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 2 — BUFFER
    //  Valeurs live + "stable depuis Xs" + GO → MARCHE
    // ----------------------------------------------------------
    case ETAT_BUFFER:
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
        compteurErreursSHT35 = 0;
        compteurErreursSCD40 = 0;
        etatThermiqueActuel = -1;
        etat = ETAT_MARCHE;
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 3 — MARCHE
    //  Régulation active + chrono transport + GO 2s → PAUSE
    // ----------------------------------------------------------
    case ETAT_MARCHE:
      if (temperature >= TEMP_SECURITE) {
        signalerErreur("TEMP CRITIQUE");
        break;
      }

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

    // ----------------------------------------------------------
    //  ÉTAT 4 — PAUSE
    //  Actionneurs OFF, GO court → MARCHE, GO 3s → BUFFER
    // ----------------------------------------------------------
    case ETAT_PAUSE:
      if (etatThermiqueActuel != THERM_NEUTRE) {
        digitalWrite(BROCHE_PELTIER, LOW);
        digitalWrite(BROCHE_VENTILATEUR, LOW);
        etatThermiqueActuel = THERM_NEUTRE;
        Serial.println("[PAUSE] Actionneurs OFF");
      }

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

    // ----------------------------------------------------------
    //  ÉTAT 5 — ERREUR
    //  Tout éteint one-shot, LED clignote, cause affichée
    //  GO 1s → restart
    // ----------------------------------------------------------
    case ETAT_ERREUR:
      if (!erreurInitialisee) {
        toutEteindre();
        erreurInitialisee = true;
        Serial.println("[ERREUR] Actionneurs OFF");
      }

      {
        static unsigned long dernierBlink = 0;
        static bool ledActive = false;
        if (millis() - dernierBlink >= 500) {
          ledActive = !ledActive;
          digitalWrite(BROCHE_LED_ROUGE, ledActive ? HIGH : LOW);
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
//  SCD40 — Démarrage mesure périodique
// =============================================================
void scd40Demarrer() {
  Wire.beginTransmission(ADRESSE_SCD40);
  Wire.write((uint8_t)(SCD40_CMD_START_MEAS >> 8));
  Wire.write((uint8_t)(SCD40_CMD_START_MEAS & 0xFF));
  uint8_t err = Wire.endTransmission();
  if (err == 0) {
    scd40Demarre = true;
    Serial.println("[SCD40] Démarré");
  } else {
    scd40Demarre = false;
    Serial.printf("[SCD40] ABSENT au démarrage (err=%d)\n", err);
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
//  SCD40 — Lecture mesure (I2C brut)
//
//  Retourne true si lecture et CRC valides.
//  co2_out en ppm, temp_out en °C, hum_out en %
// =============================================================
bool scd40Lire(float& co2_out, float& temp_out, float& hum_out) {
  // Envoyer commande lecture
  Wire.beginTransmission(ADRESSE_SCD40);
  Wire.write((uint8_t)(SCD40_CMD_READ_MEAS >> 8));
  Wire.write((uint8_t)(SCD40_CMD_READ_MEAS & 0xFF));
  if (Wire.endTransmission() != 0) return false;
  delay(1);

  // Lire 9 octets
  Wire.requestFrom((uint8_t)ADRESSE_SCD40, (uint8_t)9);
  if (Wire.available() < 9) return false;

  uint8_t data[9];
  for (uint8_t i = 0; i < 9; i++) data[i] = Wire.read();

  // Vérification CRC
  if (calculerCRC(data, 2) != data[2]) return false;
  if (calculerCRC(data + 3, 2) != data[5]) return false;
  if (calculerCRC(data + 6, 2) != data[8]) return false;

  // Conversion selon datasheet SCD40
  uint16_t raw_co2 = ((uint16_t)data[0] << 8) | data[1];
  uint16_t raw_temp = ((uint16_t)data[3] << 8) | data[4];
  uint16_t raw_hum = ((uint16_t)data[6] << 8) | data[7];

  co2_out = (float)raw_co2;
  temp_out = -45.0f + 175.0f * (raw_temp / 65535.0f);
  hum_out = 100.0f * (raw_hum / 65535.0f);
  return true;
}

// =============================================================
//  LECTURE CAPTEURS — toutes les 2s
//
//  SHT35 : température + humidité + alerte HR
//  SCD40 : CO₂ (ppm → %)
//  Détection perte à chaud en MARCHE :
//    3 NaN consécutifs → signalerErreur
//  En PAUSE : NaN tolérés (enceinte ouverte intentionnellement)
// =============================================================
void lireCapteurs() {

  // --- SHT35 ---
  if (sht35Operationnel) {
    float t = sht35.readTemperature();
    float h = sht35.readHumidity();

    if (!isnan(t) && !isnan(h)) {
      temperature = t;
      humidite = h;
      compteurErreursSHT35 = 0;

      // Alerte humidité
      if (humidite < SEUIL_HR_ALERTE) {
        if (!alerteHR) {
          alerteHR = true;
          Serial.printf("[HR] ALERTE HR=%.1f%%\n", humidite);
        }
      } else {
        if (alerteHR) {
          alerteHR = false;
          Serial.printf("[HR] Alerte levée HR=%.1f%%\n", humidite);
        }
      }
      Serial.printf("[SHT35] T=%.1fC  HR=%.1f%%\n", temperature, humidite);
    } else {
      compteurErreursSHT35++;
      Serial.printf("[SHT35] NaN (%d/%d)\n",
                    compteurErreursSHT35, SEUIL_ERREURS_LECTURE);
      if (compteurErreursSHT35 >= SEUIL_ERREURS_LECTURE && etat == ETAT_MARCHE) {
        signalerErreur("SHT35 perdu");
      }
    }
  }

  // --- SCD40 ---
  if (scd40Demarre) {
    float co2_ppm, temp_scd, hum_scd;
    if (scd40Lire(co2_ppm, temp_scd, hum_scd)) {
      co2 = co2_ppm / 10000.0f;  // ppm → %
      compteurErreursSCD40 = 0;
      Serial.printf("[SCD40] CO2=%.2f%%  T=%.1fC  HR=%.1f%%\n",
                    co2, temp_scd, hum_scd);
    } else {
      compteurErreursSCD40++;
      Serial.printf("[SCD40] Erreur lecture (%d/%d)\n",
                    compteurErreursSCD40, SEUIL_ERREURS_LECTURE);
      if (compteurErreursSCD40 >= SEUIL_ERREURS_LECTURE && etat == ETAT_MARCHE) {
        signalerErreur("SCD40 perdu");
      }
    }
  }

  mettreAJourLedI2C();
}

// =============================================================
//  SCAN I2C — tentatives multiples
// =============================================================
bool scannerAdresse(uint8_t adresse, uint8_t tentatives) {
  for (uint8_t i = 0; i < tentatives; i++) {
    Wire.beginTransmission(adresse);
    if (Wire.endTransmission() == 0) return true;
    if (i < tentatives - 1) delay(DELAI_RETRY_I2C);
  }
  return false;
}

// =============================================================
//  SCAN CAPTEURS I2C
//  Met à jour scanSHT35OK, scanSCD40OK, flags opérationnels
//  Réinitialise SCD40 si détecté mais pas encore démarré
// =============================================================
void scannerCapteursI2C() {
  Serial.println("[I2C] Scan...");

  // SHT35
  scanSHT35OK = scannerAdresse(ADRESSE_SHT35, TENTATIVES_I2C);
  if (scanSHT35OK && !sht35Operationnel) {
    // Réinitialiser SHT35 s'il revient
    sht35Operationnel = sht35.begin(ADRESSE_SHT35);
  } else if (!scanSHT35OK) {
    sht35Operationnel = false;
  }
  Serial.printf("[I2C] SHT35 → %s\n", scanSHT35OK ? "OK" : "ABSENT");

  // SCD40
  scanSCD40OK = scannerAdresse(ADRESSE_SCD40, TENTATIVES_I2C);
  if (scanSCD40OK && !scd40Demarre) {
    // Démarrer SCD40 s'il revient
    scd40Demarrer();
  } else if (!scanSCD40OK) {
    scd40Demarre = false;
  }
  Serial.printf("[I2C] SCD40 → %s\n", scanSCD40OK ? "OK" : "ABSENT");

  mettreAJourLedI2C();
  afficherResultatI2C(scanSHT35OK, scanSCD40OK);
}

// =============================================================
//  RÉGULATION THERMIQUE — état mémorisé
//  digitalWrite émis UNIQUEMENT au changement d'état thermique
// =============================================================
void regulerTemperature() {
  int nouvelEtat = (temperature < SEUIL_CHAUFFE) ? THERM_CHAUFFE : (temperature > SEUIL_REFROIDIT) ? THERM_DISSIPE
                                                                                                   : THERM_NEUTRE;

  if (nouvelEtat == etatThermiqueActuel) return;
  etatThermiqueActuel = nouvelEtat;

  switch (nouvelEtat) {
    case THERM_CHAUFFE:
      digitalWrite(BROCHE_PELTIER, HIGH);
      digitalWrite(BROCHE_VENTILATEUR, HIGH);
      Serial.printf("[THERM] Chauffe T=%.1fC\n", temperature);
      break;
    case THERM_DISSIPE:
      digitalWrite(BROCHE_PELTIER, LOW);
      digitalWrite(BROCHE_VENTILATEUR, HIGH);
      Serial.printf("[THERM] Dissipe T=%.1fC\n", temperature);
      break;
    case THERM_NEUTRE:
      digitalWrite(BROCHE_PELTIER, LOW);
      digitalWrite(BROCHE_VENTILATEUR, LOW);
      Serial.printf("[THERM] Neutre  T=%.1fC\n", temperature);
      break;
  }
}

// =============================================================
//  RÉGULATION CO₂ — injection impulsionnelle non bloquante
// =============================================================
void miseAJourCO2() {
  unsigned long maintenant = millis();
  switch (etatCO2) {
    case CO2_IDLE:
      if (maintenant - dernierCO2 >= INTERVALLE_CO2) {
        dernierCO2 = maintenant;
        if (co2 < SEUIL_BAS_CO2) {
          digitalWrite(BROCHE_SOLENOIDE, HIGH);
          timerCO2 = maintenant;
          etatCO2 = CO2_INJECTING;
          Serial.printf("[CO2] Injection %.2f%%\n", co2);
        } else if (co2 > SEUIL_HAUT_CO2) {
          Serial.printf("[CO2] Trop élevé %.2f%% — dilution\n", co2);
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
//  TOUT ÉTEINDRE
// =============================================================
void toutEteindre() {
  digitalWrite(BROCHE_PELTIER, LOW);
  digitalWrite(BROCHE_VENTILATEUR, LOW);
  digitalWrite(BROCHE_SOLENOIDE, LOW);
  digitalWrite(BROCHE_LED_ROUGE, LOW);
  digitalWrite(BROCHE_LED_I2C, LOW);
  etatThermiqueActuel = -1;
}

// =============================================================
//  SIGNALER ERREUR — mémorise le message pour l'affichage
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
//  AFFICHAGE RÉSULTAT I2C
// =============================================================
void afficherResultatI2C(bool sht35_ok, bool scd40_ok) {
  ecran.fillScreen(0x1E9F);
  ecran.setTextDatum(MC_DATUM);

  ecran.setTextColor(TFT_WHITE, 0x1E9F);
  ecran.setTextSize(2);
  ecran.drawString("Capteurs I2C", 120, 20);

  ecran.setTextSize(1);

  // SHT35
  ecran.setTextColor(sht35_ok ? TFT_GREEN : TFT_RED, 0x1E9F);
  ecran.drawString(sht35_ok ? "SHT35 [0x44]  OK"
                            : "SHT35 [0x44]  ABSENT",
                   120, 70);

  // SCD40
  ecran.setTextColor(scd40_ok ? TFT_GREEN : TFT_RED, 0x1E9F);
  ecran.drawString(scd40_ok ? "SCD40 [0x62]  OK"
                            : "SCD40 [0x62]  ABSENT",
                   120, 95);

  // Résultat global
  ecran.setTextSize(2);
  bool tousOK = sht35_ok && scd40_ok;
  ecran.setTextColor(tousOK ? TFT_GREEN : TFT_RED, 0x1E9F);
  ecran.drawString(tousOK ? "SYSTEME OK" : "ATTENTE...", 120, 145);

  // Timer stabilité
  ecran.setTextSize(1);
  ecran.setTextColor(TFT_WHITE, 0x1E9F);
  if (tousOK && tempsCapteursOK > 0) {
    char buf[32];
    uint32_t sec = (millis() - tempsCapteursOK) / 1000;
    snprintf(buf, sizeof(buf), "Stable depuis %lus", sec);
    ecran.drawString(buf, 120, 190);
    ecran.drawString("GO pour continuer", 120, 215);
  } else {
    ecran.drawString("Brancher SHT35 + SCD40", 120, 190);
    ecran.drawString("GO ignore si capteur absent", 120, 215);
  }
}

// =============================================================
//  BARRE DE PROGRESSION TEMPÉRATURE
//
//  Dessine une barre horizontale colorée entre BARRE_TEMP_MIN
//  et BARRE_TEMP_MAX avec repères SEUIL_CHAUFFE / SEUIL_REFROIDIT
//
//  Paramètres :
//    x, y   : coin supérieur gauche
//    w, h   : largeur et hauteur en pixels
//    fond   : couleur de fond de l'écran (pour effacement)
// =============================================================
void dessinBarreTemp(int x, int y, int w, int h, uint16_t fond) {
  // Fond de la barre (gris foncé)
  ecran.fillRect(x, y, w, h, 0x4208);

  // Position de la température dans la plage
  float ratio = (temperature - BARRE_TEMP_MIN) / (BARRE_TEMP_MAX - BARRE_TEMP_MIN);
  ratio = constrain(ratio, 0.0f, 1.0f);
  int largeurRemplie = (int)(ratio * w);

  // Couleur selon état thermique
  uint16_t couleur;
  if (etatThermiqueActuel == THERM_CHAUFFE) couleur = 0xFD20;       // Orange
  else if (etatThermiqueActuel == THERM_DISSIPE) couleur = 0xF800;  // Rouge
  else couleur = TFT_GREEN;

  ecran.fillRect(x, y, largeurRemplie, h, couleur);

  // Repère SEUIL_CHAUFFE (36.5°C)
  float ratioChauffe = (SEUIL_CHAUFFE - BARRE_TEMP_MIN) / (BARRE_TEMP_MAX - BARRE_TEMP_MIN);
  int xChauffe = x + (int)(ratioChauffe * w);
  ecran.drawFastVLine(xChauffe, y - 2, h + 4, TFT_WHITE);

  // Repère SEUIL_REFROIDIT (37.5°C)
  float ratioRefroidit = (SEUIL_REFROIDIT - BARRE_TEMP_MIN) / (BARRE_TEMP_MAX - BARRE_TEMP_MIN);
  int xRefroidit = x + (int)(ratioRefroidit * w);
  ecran.drawFastVLine(xRefroidit, y - 2, h + 4, TFT_WHITE);

  // Repère CONSIGNE (37.0°C) — tiret central
  float ratioConsigne = (CONSIGNE_TEMP - BARRE_TEMP_MIN) / (BARRE_TEMP_MAX - BARRE_TEMP_MIN);
  int xConsigne = x + (int)(ratioConsigne * w);
  ecran.drawFastVLine(xConsigne, y - 4, h + 8, TFT_YELLOW);

  // Bordure
  ecran.drawRect(x, y, w, h, TFT_WHITE);
}

// =============================================================
//  AFFICHAGE ÉCRAN TFT — dispatcher
//  240 × 280 px portrait — centre x = 120
// =============================================================
void afficherEcran(int e) {
  ecran.fillScreen(TFT_BLACK);
  ecran.setTextDatum(MC_DATUM);

  switch (e) {

    // --- INIT ---
    case ETAT_INIT:
      ecran.fillScreen(TFT_NAVY);
      ecran.setTextColor(TFT_WHITE, TFT_NAVY);
      ecran.setTextSize(2);
      ecran.drawString("PB-0326", 120, 60);
      ecran.setTextSize(1);
      ecran.drawString("Incubateur transportable", 120, 110);
      ecran.drawString("HEPIA HES-SO  Groupe 3", 120, 135);
      ecran.drawString("v3.8", 120, 160);
      ecran.drawString("Initialisation...", 120, 200);
      break;

      // --- CHECK ---
    case ETAT_CHECK:
      // Écran géré par afficherResultatI2C() dans scannerCapteursI2C()
      // Ne rien dessiner ici pour éviter le flash "Scan I2C..."
      return;

    // --- BUFFER ---
    case ETAT_BUFFER:
      {
        ecran.fillScreen(0xFD20);
        ecran.setTextColor(TFT_BLACK, 0xFD20);
        ecran.setTextSize(2);
        ecran.drawString("PRET", 120, 25);
        ecran.setTextSize(1);
        char buf[32];

        snprintf(buf, sizeof(buf), "T:   %.1f C", temperature);
        ecran.drawString(buf, 120, 75);

        snprintf(buf, sizeof(buf), "HR:  %.1f %%", humidite);
        ecran.drawString(buf, 120, 95);

        snprintf(buf, sizeof(buf), "CO2: %.2f %%", co2);
        ecran.drawString(buf, 120, 115);

        // Timer stabilité capteurs
        if (tempsCapteursOK > 0) {
          uint32_t sec = (millis() - tempsCapteursOK) / 1000;
          snprintf(buf, sizeof(buf), "Stable depuis %lus", sec);
          ecran.setTextColor(TFT_GREEN, 0xFD20);
          ecran.drawString(buf, 120, 148);
        }

        ecran.setTextColor(TFT_BLACK, 0xFD20);
        ecran.drawString("Fermez l'enceinte", 120, 185);
        ecran.drawString("GO pour demarrer", 120, 205);
        break;
      }

    // --- MARCHE ---
    case ETAT_MARCHE:
      {
        ecran.fillScreen(0x07E0);
        ecran.setTextColor(TFT_BLACK, 0x07E0);
        ecran.setTextSize(2);
        ecran.drawString("EN MARCHE", 120, 18);
        ecran.setTextSize(1);
        char ligne[32];

        // Chrono transport HH:MM:SS
        unsigned long elapsed = (millis() - tempsDebutMarche) / 1000;
        uint8_t hh = elapsed / 3600;
        uint8_t mm = (elapsed % 3600) / 60;
        uint8_t ss = elapsed % 60;
        snprintf(ligne, sizeof(ligne), "Transport: %02d:%02d:%02d", hh, mm, ss);
        ecran.drawString(ligne, 120, 42);

        // Température + valeur numérique
        snprintf(ligne, sizeof(ligne), "T: %.1f / %.1f C", temperature, CONSIGNE_TEMP);
        ecran.drawString(ligne, 120, 68);

        // Barre de progression température
        dessinBarreTemp(20, 80, 200, 12, 0x07E0);

        // Légende barre
        ecran.setTextSize(1);
        ecran.setTextColor(TFT_BLACK, 0x07E0);
        ecran.drawString("35", 20, 98);
        ecran.drawString("37", 108, 98);
        ecran.drawString("39", 210, 98);

        // Humidité
        if (alerteHR) {
          ecran.setTextColor(TFT_RED, 0x07E0);
          snprintf(ligne, sizeof(ligne), "HR: %.1f %% ALERTE", humidite);
        } else {
          snprintf(ligne, sizeof(ligne), "HR: %.1f %%", humidite);
        }
        ecran.drawString(ligne, 120, 115);
        ecran.setTextColor(TFT_BLACK, 0x07E0);

        // CO₂ avec indicateur état injection
        const char* indCO2 = (etatCO2 == CO2_INJECTING) ? " [INJ]" : (etatCO2 == CO2_STABILIZING) ? " [STB]"
                                                                                                  : "";
        snprintf(ligne, sizeof(ligne), "CO2: %.2f %%%s", co2, indCO2);
        ecran.drawString(ligne, 120, 135);

        // État Peltier
        const char* labelTherm =
          (etatThermiqueActuel == THERM_CHAUFFE) ? "Chauffe + ventil" : (etatThermiqueActuel == THERM_DISSIPE) ? "Dissipe (ventil)"
                                                                                                               : "Neutre";
        ecran.drawString(labelTherm, 120, 155);

        // Compteurs erreurs si actifs
        if (compteurErreursSHT35 > 0 || compteurErreursSCD40 > 0) {
          ecran.setTextColor(TFT_YELLOW, 0x07E0);
          snprintf(ligne, sizeof(ligne), "Err SHT35:%d SCD40:%d",
                   compteurErreursSHT35, compteurErreursSCD40);
          ecran.drawString(ligne, 120, 175);
          ecran.setTextColor(TFT_BLACK, 0x07E0);
        }

        ecran.drawString("GO 2s pour pause", 120, 258);
        break;
      }

    // --- PAUSE ---
    case ETAT_PAUSE:
      {
        ecran.fillScreen(0xFC60);
        ecran.setTextColor(TFT_BLACK, 0xFC60);
        ecran.setTextSize(2);
        ecran.drawString("EN PAUSE", 120, 22);
        ecran.setTextSize(1);
        char buf[32];

        ecran.drawString("Regulation suspendue", 120, 58);
        ecran.drawString("Actionneurs OFF", 120, 75);

        snprintf(buf, sizeof(buf), "T:   %.1f C", temperature);
        ecran.drawString(buf, 120, 110);

        snprintf(buf, sizeof(buf), "HR:  %.1f %%", humidite);
        ecran.drawString(buf, 120, 130);

        snprintf(buf, sizeof(buf), "CO2: %.2f %%", co2);
        ecran.drawString(buf, 120, 150);

        // Chrono arrêté à la valeur courante
        if (tempsDebutMarche > 0) {
          unsigned long elapsed = (millis() - tempsDebutMarche) / 1000;
          uint8_t hh = elapsed / 3600;
          uint8_t mm = (elapsed % 3600) / 60;
          uint8_t ss = elapsed % 60;
          snprintf(buf, sizeof(buf), "Transport: %02d:%02d:%02d", hh, mm, ss);
          ecran.drawString(buf, 120, 175);
        }

        ecran.drawString("GO court  → reprendre", 120, 215);
        ecran.drawString("GO 3s     → retour PRET", 120, 235);
        break;
      }

    // --- ERREUR ---
    case ETAT_ERREUR:
      ecran.fillScreen(TFT_RED);
      ecran.setTextColor(TFT_WHITE, TFT_RED);
      ecran.setTextSize(2);
      ecran.drawString("ERREUR", 120, 60);
      ecran.setTextSize(1);
      // Affichage de la cause
      ecran.drawString(messageErreur, 120, 110);
      ecran.drawString("Systeme arrete", 120, 135);
      ecran.drawString("Actionneurs OFF", 120, 155);
      ecran.drawString("GO 1s → redemarrer", 120, 205);
      break;
  }
}
