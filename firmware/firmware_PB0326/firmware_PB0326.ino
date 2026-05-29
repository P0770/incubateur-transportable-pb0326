// =============================================================
//  PB-0326 — Incubateur transportable pour cellules mammifères
//  HEPIA HES-SO — Groupe 3 : Lyla Founas, Mattéo Pozzo di Borgo
//  Version : 3.9 — Mai 2026
// =============================================================
//
//  MACHINE À ÉTATS v3.9 :
//    0 - INIT    : splash 3s automatique
//    1 - CHECK   : scan I2C toutes les 2s
//                  SHT35 + SCD40 OK → GO → BUFFER
//                  Capteur absent → reste en CHECK, pas d'ERREUR
//                  Rebranché → réinitialisation propre
//    2 - BUFFER  : valeurs live + timer stable + GO → MARCHE
//    3 - MARCHE  : régulation active + barres + chrono HH:MM:SS
//                  GO 2s → PAUSE
//    4 - PAUSE   : actionneurs OFF + GO court → MARCHE, GO 3s → BUFFER
//    5 - ERREUR  : tout éteint, cause affichée, GO 1s → restart
//                  lireCapteurs() bloquée → LED verte reste OFF
//
//  CORRECTIONS v3.9 :
//    - sht35.begin() sur objet existant → création nouvel objet
//      pour réinitialisation propre après rebranchement
//    - SCD40 rebranché : flag scd40EnChauffe + timer 5s avant lectures
//      évite faux NaN → ERREUR au redémarrage capteur
//    - lireCapteurs() bloquée en ETAT_ERREUR → LED verte reste OFF
//    - Barres humidité et CO₂ ajoutées en MARCHE et PAUSE
//    - ETAT_CHECK : suppression placeholder "Scan I2C..."
//      afficherEcran(CHECK) retourne immédiatement (return)
//      → plus de flash visuel entre scans
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
#define BROCHE_PELTIER      32
#define BROCHE_VENTILATEUR  14
#define BROCHE_SOLENOIDE    27
#define BROCHE_LED_ROUGE    19
#define BROCHE_LED_I2C       4
#define BROCHE_BOUTON_GO     5

// =============================================================
//  CONSIGNES BIOLOGIQUES
// =============================================================
#define CONSIGNE_TEMP    37.0f
#define SEUIL_CHAUFFE    36.5f
#define SEUIL_REFROIDIT  37.5f
#define TEMP_SECURITE    40.0f
#define CONSIGNE_CO2      5.0f
#define SEUIL_BAS_CO2     4.7f
#define SEUIL_HAUT_CO2    5.3f
#define SEUIL_HR_ALERTE  85.0f

// Plages barres de progression
#define BARRE_TEMP_MIN  35.0f   // °C
#define BARRE_TEMP_MAX  39.0f   // °C
#define BARRE_HR_MIN     0.0f   // %
#define BARRE_HR_MAX   100.0f   // %
#define BARRE_CO2_MIN    0.0f   // %
#define BARRE_CO2_MAX   10.0f   // %

// =============================================================
//  PARAMÈTRES CO₂
// =============================================================
#define DUREE_INJECT_MS      200
#define DUREE_STABILISATION  30000
#define INTERVALLE_CO2        5000
#define DUREE_CHAUFFE_SCD40   5000  // Attente (ms) après restart SCD40

// =============================================================
//  PARAMÈTRES DIVERS
// =============================================================
#define INTERVALLE_ECRAN         2000
#define INTERVALLE_LECTURE       2000
#define INTERVALLE_SCAN_I2C      2000
#define DUREE_INIT               3000
#define DUREE_ANTIREBOND           50
#define DUREE_APPUI_PAUSE        2000
#define DUREE_APPUI_BUFFER_PAUSE 3000
#define DUREE_APPUI_RESET        1000
#define TENTATIVES_I2C              3
#define DELAI_RETRY_I2C            50
#define SEUIL_ERREURS_LECTURE       3

// =============================================================
//  ÉTATS
// =============================================================
#define ETAT_INIT    0
#define ETAT_CHECK   1
#define ETAT_BUFFER  2
#define ETAT_MARCHE  3
#define ETAT_PAUSE   4
#define ETAT_ERREUR  5

// =============================================================
//  ÉTATS THERMIQUES
// =============================================================
#define THERM_NEUTRE  0
#define THERM_CHAUFFE 1
#define THERM_DISSIPE 2

// =============================================================
//  ÉTATS CO₂
// =============================================================
#define CO2_IDLE        0
#define CO2_INJECTING   1
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
#define SCD40_CMD_READ_MEAS  0xEC05

// =============================================================
//  OBJETS
// =============================================================
TFT_eSPI       ecran = TFT_eSPI();
Adafruit_SHT31 sht35 = Adafruit_SHT31();

// =============================================================
//  VARIABLES GLOBALES
// =============================================================

// --- Machine à états ---
int  etat          = ETAT_INIT;
int  etatPrecedent = -1;
char messageErreur[32] = "";

// --- Valeurs capteurs ---
float temperature = 25.0f;
float humidite    = 50.0f;
float co2         =  0.0f;  // en % (ppm / 10000)

// --- Timers ---
unsigned long derniereLecture   = 0;
unsigned long dernierEcran      = 0;
unsigned long tempsInit         = 0;
unsigned long dernierScanI2C    = 0;
unsigned long tempsDebutMarche  = 0;
unsigned long tempsCapteursOK   = 0;
unsigned long tempsRedemarrageSCD40 = 0;  // Timer chauffe SCD40 après restart

// --- CO₂ ---
int           etatCO2    = CO2_IDLE;
unsigned long timerCO2   = 0;
unsigned long dernierCO2 = 0;

// --- Flags capteurs ---
bool sht35Operationnel = false;
bool scd40Demarre      = false;
bool scd40EnChauffe    = false;  // true pendant les 5s après restart SCD40
                                  // lectures ignorées pendant cette période
bool alerteHR          = false;

// --- Scan I2C ---
bool scanEffectue = false;
bool scanSHT35OK  = false;
bool scanSCD40OK  = false;

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
  bool          etatPrecedent    = false;
  bool          appuiDetecte     = false;
  unsigned long tempsAppui       = 0;
  bool          appuiCourtValide = false;
} bouton;

void mettreAJourBouton() {
  bool lu = (digitalRead(BROCHE_BOUTON_GO) == LOW);
  if (lu && !bouton.etatPrecedent) {
    bouton.tempsAppui       = millis();
    bouton.appuiDetecte     = true;
    bouton.appuiCourtValide = false;
  }
  if (!lu && bouton.etatPrecedent) {
    if (bouton.appuiDetecte &&
        (millis() - bouton.tempsAppui) >= DUREE_ANTIREBOND &&
        (millis() - bouton.tempsAppui) <  DUREE_APPUI_PAUSE) {
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
  return bouton.etatPrecedent &&
         bouton.appuiDetecte  &&
         (millis() - bouton.tempsAppui) >= duree;
}

// =============================================================
//  LED I2C — ON si SHT35 ET SCD40 opérationnels
// =============================================================
void mettreAJourLedI2C() {
  digitalWrite(BROCHE_LED_I2C,
    (sht35Operationnel && scd40Demarre && !scd40EnChauffe) ? HIGH : LOW);
}

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("=== PB-0326 Firmware v3.9 ===");

  pinMode(BROCHE_PELTIER,     OUTPUT);
  pinMode(BROCHE_VENTILATEUR, OUTPUT);
  pinMode(BROCHE_SOLENOIDE,   OUTPUT);
  pinMode(BROCHE_LED_ROUGE,   OUTPUT);
  pinMode(BROCHE_LED_I2C,     OUTPUT);
  pinMode(BROCHE_BOUTON_GO,   INPUT_PULLUP);

  toutEteindre();

  Wire.begin(21, 22);
  delay(100);

  // Tentative init SHT35
  if (sht35.begin(ADRESSE_SHT35)) {
    sht35Operationnel = true;
    Serial.println("[SHT35] OK");
  } else {
    Serial.println("[SHT35] ABSENT au démarrage");
  }

  // Tentative init SCD40
  scd40Demarrer();

  mettreAJourLedI2C();

  // Écran TFT — broches configurées dans User_Setup.h
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

  // Chauffe SCD40 — fin de période d'attente après restart
  if (scd40EnChauffe && (millis() - tempsRedemarrageSCD40 >= DUREE_CHAUFFE_SCD40)) {
    scd40EnChauffe = false;
    Serial.println("[SCD40] Chauffe terminée — lectures actives");
    mettreAJourLedI2C();
  }

  // Lecture capteurs toutes les 2s — BLOQUÉE en ETAT_ERREUR
  if (etat != ETAT_ERREUR &&
      millis() - derniereLecture >= INTERVALLE_LECTURE) {
    lireCapteurs();
    derniereLecture = millis();
  }

  // Refresh TFT au changement d'état
  if (etat != etatPrecedent) {
    afficherEcran(etat);
    etatPrecedent = etat;
    if (etat != ETAT_CHECK)  scanEffectue    = false;
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
    //  Capteur absent → reste en CHECK, affiche ABSENT, pas d'ERREUR.
    //  Rebranché → réinitialisation propre (nouvel objet SHT35).
    //  SCD40 rebranché → flag scd40EnChauffe → lectures ignorées 5s.
    //  GO ignoré si un capteur manque ou SCD40 en chauffe.
    // ----------------------------------------------------------
    case ETAT_CHECK:
      if (!scanEffectue ||
          (millis() - dernierScanI2C >= INTERVALLE_SCAN_I2C)) {
        scannerCapteursI2C();
        dernierScanI2C = millis();
        scanEffectue   = true;

        if (!scanSHT35OK || !scanSCD40OK || scd40EnChauffe) {
          tempsCapteursOK = 0;
        } else if (tempsCapteursOK == 0) {
          tempsCapteursOK = millis();
        }
      }

      // GO → BUFFER si tout OK et SCD40 pas en chauffe
      if (boutonAppuiCourt()) {
        if (scanSHT35OK && scanSCD40OK && !scd40EnChauffe) {
          etat = ETAT_BUFFER;
        } else {
          Serial.println("[CHECK] GO ignoré — capteurs non prêts");
        }
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 2 — BUFFER
    // ----------------------------------------------------------
    case ETAT_BUFFER:
      if (millis() - dernierEcran >= INTERVALLE_ECRAN) {
        afficherEcran(ETAT_BUFFER);
        dernierEcran = millis();
      }

      if (boutonAppuiCourt()) {
        Serial.println("[BUFFER] GO → MARCHE");
        tempsDebutMarche     = millis();
        dernierCO2           = millis();
        dernierEcran         = millis();
        etatCO2              = CO2_IDLE;
        compteurErreursSHT35 = 0;
        compteurErreursSCD40 = 0;
        etatThermiqueActuel  = -1;
        etat                 = ETAT_MARCHE;
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 3 — MARCHE
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
    // ----------------------------------------------------------
    case ETAT_PAUSE:
      if (etatThermiqueActuel != THERM_NEUTRE) {
        digitalWrite(BROCHE_PELTIER,     LOW);
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
        etatThermiqueActuel     = -1;
        compteurErreursSHT35    = 0;
        compteurErreursSCD40    = 0;
        dernierEcran            = millis();
        etat                    = ETAT_BUFFER;
      } else if (boutonAppuiCourt()) {
        Serial.println("[PAUSE] GO → MARCHE");
        etatThermiqueActuel  = -1;
        compteurErreursSHT35 = 0;
        compteurErreursSCD40 = 0;
        dernierEcran         = millis();
        etat                 = ETAT_MARCHE;
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 5 — ERREUR
    //  lireCapteurs() bloquée → LED verte reste OFF
    //  Cause affichée sur écran
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
        static bool          ledActive    = false;
        if (millis() - dernierBlink >= 500) {
          ledActive    = !ledActive;
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
//
//  Envoie START_PERIODIC_MEASUREMENT.
//  Si succès : scd40Demarre = true, scd40EnChauffe = true
//  Le flag scd40EnChauffe reste true pendant DUREE_CHAUFFE_SCD40 (5s)
//  pendant laquelle les lectures sont ignorées (données non valides).
// =============================================================
void scd40Demarrer() {
  Wire.beginTransmission(ADRESSE_SCD40);
  Wire.write((uint8_t)(SCD40_CMD_START_MEAS >> 8));
  Wire.write((uint8_t)(SCD40_CMD_START_MEAS & 0xFF));
  uint8_t err = Wire.endTransmission();
  if (err == 0) {
    scd40Demarre          = true;
    scd40EnChauffe        = true;
    tempsRedemarrageSCD40 = millis();
    Serial.println("[SCD40] Démarré — chauffe 5s");
  } else {
    scd40Demarre   = false;
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
//  SCD40 — Lecture mesure
//  Retourne true si CRC valides, false sinon
// =============================================================
bool scd40Lire(float& co2_out, float& temp_out, float& hum_out) {
  Wire.beginTransmission(ADRESSE_SCD40);
  Wire.write((uint8_t)(SCD40_CMD_READ_MEAS >> 8));
  Wire.write((uint8_t)(SCD40_CMD_READ_MEAS & 0xFF));
  if (Wire.endTransmission() != 0) return false;
  delay(1);

  Wire.requestFrom((uint8_t)ADRESSE_SCD40, (uint8_t)9);
  if (Wire.available() < 9) return false;

  uint8_t data[9];
  for (uint8_t i = 0; i < 9; i++) data[i] = Wire.read();

  if (calculerCRC(data,     2) != data[2]) return false;
  if (calculerCRC(data + 3, 2) != data[5]) return false;
  if (calculerCRC(data + 6, 2) != data[8]) return false;

  uint16_t raw_co2  = ((uint16_t)data[0] << 8) | data[1];
  uint16_t raw_temp = ((uint16_t)data[3] << 8) | data[4];
  uint16_t raw_hum  = ((uint16_t)data[6] << 8) | data[7];

  co2_out  = (float)raw_co2;
  temp_out = -45.0f + 175.0f * (raw_temp / 65535.0f);
  hum_out  = 100.0f * (raw_hum  / 65535.0f);
  return true;
}

// =============================================================
//  LECTURE CAPTEURS — toutes les 2s (bloquée en ETAT_ERREUR)
//
//  SHT35 : température + humidité + alerte HR intégrée
//  SCD40 : CO₂ — ignoré si scd40EnChauffe
//  3 NaN consécutifs en MARCHE → signalerErreur
//  En PAUSE : NaN tolérés (enceinte ouverte)
// =============================================================
void lireCapteurs() {

  // --- SHT35 ---
  if (sht35Operationnel) {
    float t = sht35.readTemperature();
    float h = sht35.readHumidity();

    if (!isnan(t) && !isnan(h)) {
      temperature          = t;
      humidite             = h;
      compteurErreursSHT35 = 0;

      // Alerte humidité intégrée
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
      Serial.printf("[SHT35] NaN (%d/%d)\n",
                    compteurErreursSHT35, SEUIL_ERREURS_LECTURE);
      if (compteurErreursSHT35 >= SEUIL_ERREURS_LECTURE && etat == ETAT_MARCHE) {
        signalerErreur("SHT35 perdu");
        return;
      }
    }
  }

  // --- SCD40 — ignoré pendant la chauffe après restart ---
  if (scd40Demarre && !scd40EnChauffe) {
    float co2_ppm, temp_scd, hum_scd;
    if (scd40Lire(co2_ppm, temp_scd, hum_scd)) {
      co2                  = co2_ppm / 10000.0f;
      compteurErreursSCD40 = 0;
      Serial.printf("[SCD40] CO2=%.2f%%\n", co2);
    } else {
      compteurErreursSCD40++;
      Serial.printf("[SCD40] Erreur (%d/%d)\n",
                    compteurErreursSCD40, SEUIL_ERREURS_LECTURE);
      if (compteurErreursSCD40 >= SEUIL_ERREURS_LECTURE && etat == ETAT_MARCHE) {
        signalerErreur("SCD40 perdu");
        return;
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
//
//  SHT35 rebranché :
//    Crée un NOUVEL objet Adafruit_SHT31 pour forcer
//    la réinitialisation propre du driver.
//    sht35.begin() sur objet existant peut retourner false
//    même si le capteur répond → LED verte restait OFF.
//
//  SCD40 rebranché :
//    Appelle scd40Demarrer() → flag scd40EnChauffe = true
//    Lectures ignorées pendant 5s → pas de faux NaN.
// =============================================================
void scannerCapteursI2C() {
  Serial.println("[I2C] Scan...");

  // --- SHT35 ---
  scanSHT35OK = scannerAdresse(ADRESSE_SHT35, TENTATIVES_I2C);

  if (scanSHT35OK && !sht35Operationnel) {
    // Rebranché : créer un nouvel objet pour forcer réinit propre
    sht35 = Adafruit_SHT31();
    sht35Operationnel = sht35.begin(ADRESSE_SHT35);
    if (sht35Operationnel) {
      Serial.println("[SHT35] Réinitialisé OK");
    } else {
      Serial.println("[SHT35] Réinit échouée");
      scanSHT35OK = false;
    }
  } else if (!scanSHT35OK) {
    sht35Operationnel = false;
  }
  Serial.printf("[I2C] SHT35 → %s\n", scanSHT35OK ? "OK" : "ABSENT");

  // --- SCD40 ---
  scanSCD40OK = scannerAdresse(ADRESSE_SCD40, TENTATIVES_I2C);

  if (scanSCD40OK && !scd40Demarre) {
    // Rebranché : démarrer avec période de chauffe
    scd40Demarrer();
  } else if (!scanSCD40OK) {
    scd40Demarre   = false;
    scd40EnChauffe = false;
  }
  Serial.printf("[I2C] SCD40 → %s%s\n",
                scanSCD40OK ? "OK" : "ABSENT",
                scd40EnChauffe ? " (chauffe)" : "");

  mettreAJourLedI2C();
  afficherResultatI2C(scanSHT35OK, scanSCD40OK);
}

// =============================================================
//  RÉGULATION THERMIQUE — état mémorisé
//  digitalWrite émis UNIQUEMENT au changement d'état
// =============================================================
void regulerTemperature() {
  int nouvelEtat = (temperature < SEUIL_CHAUFFE)  ? THERM_CHAUFFE :
                  (temperature > SEUIL_REFROIDIT) ? THERM_DISSIPE : THERM_NEUTRE;

  if (nouvelEtat == etatThermiqueActuel) return;
  etatThermiqueActuel = nouvelEtat;

  switch (nouvelEtat) {
    case THERM_CHAUFFE:
      digitalWrite(BROCHE_PELTIER,     HIGH);
      digitalWrite(BROCHE_VENTILATEUR, HIGH);
      Serial.printf("[THERM] Chauffe T=%.1fC\n", temperature);
      break;
    case THERM_DISSIPE:
      digitalWrite(BROCHE_PELTIER,     LOW);
      digitalWrite(BROCHE_VENTILATEUR, HIGH);
      Serial.printf("[THERM] Dissipe T=%.1fC\n", temperature);
      break;
    case THERM_NEUTRE:
      digitalWrite(BROCHE_PELTIER,     LOW);
      digitalWrite(BROCHE_VENTILATEUR, LOW);
      Serial.printf("[THERM] Neutre  T=%.1fC\n", temperature);
      break;
  }
}

// =============================================================
//  RÉGULATION CO₂
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
          etatCO2  = CO2_INJECTING;
          Serial.printf("[CO2] Injection %.2f%%\n", co2);
        } else if (co2 > SEUIL_HAUT_CO2) {
          Serial.printf("[CO2] Trop élevé %.2f%%\n", co2);
        }
      }
      break;
    case CO2_INJECTING:
      if (maintenant - timerCO2 >= DUREE_INJECT_MS) {
        digitalWrite(BROCHE_SOLENOIDE, LOW);
        timerCO2 = maintenant;
        etatCO2  = CO2_STABILIZING;
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
  digitalWrite(BROCHE_PELTIER,     LOW);
  digitalWrite(BROCHE_VENTILATEUR, LOW);
  digitalWrite(BROCHE_SOLENOIDE,   LOW);
  digitalWrite(BROCHE_LED_ROUGE,   LOW);
  digitalWrite(BROCHE_LED_I2C,     LOW);
  etatThermiqueActuel = -1;
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
  erreurInitialisee    = false;
  etat = ETAT_ERREUR;
  afficherEcran(ETAT_ERREUR);
}

// =============================================================
//  BARRE DE PROGRESSION GÉNÉRIQUE
//
//  Dessine une barre horizontale avec :
//    - Zone colorée proportionnelle à valeur dans [valMin, valMax]
//    - Couleur selon position par rapport à [zoneOKmin, zoneOKmax]
//    - Repères verticaux blancs aux limites de la zone OK
//    - Repère jaune à la consigne (si consigne >= 0)
//
//  Paramètres :
//    x, y          : coin supérieur gauche
//    w, h          : dimensions en pixels
//    valeur        : valeur à afficher
//    valMin/valMax : plage totale de la barre
//    zoneOKmin/max : plage acceptable (repères blancs)
//    consigne      : valeur cible (repère jaune), -1 si aucune
// =============================================================
void dessinBarre(int x, int y, int w, int h,
                 float valeur,
                 float valMin, float valMax,
                 float zoneOKmin, float zoneOKmax,
                 float consigne) {

  // Fond gris foncé
  ecran.fillRect(x, y, w, h, 0x4208);

  // Position valeur dans la plage
  float ratio = constrain((valeur - valMin) / (valMax - valMin), 0.0f, 1.0f);
  int largeur = (int)(ratio * w);

  // Couleur : vert si dans zone OK, orange si proche, rouge si hors zone
  uint16_t couleur;
  if (valeur >= zoneOKmin && valeur <= zoneOKmax) {
    couleur = TFT_GREEN;
  } else if (valeur < zoneOKmin - (zoneOKmax - zoneOKmin) * 0.2f ||
             valeur > zoneOKmax + (zoneOKmax - zoneOKmin) * 0.2f) {
    couleur = 0xF800;  // Rouge — très hors zone
  } else {
    couleur = 0xFD20;  // Orange — légèrement hors zone
  }

  ecran.fillRect(x, y, largeur, h, couleur);

  // Repère limite basse zone OK (blanc)
  float ratioOKmin = constrain((zoneOKmin - valMin) / (valMax - valMin), 0.0f, 1.0f);
  ecran.drawFastVLine(x + (int)(ratioOKmin * w), y - 2, h + 4, TFT_WHITE);

  // Repère limite haute zone OK (blanc)
  float ratioOKmax = constrain((zoneOKmax - valMin) / (valMax - valMin), 0.0f, 1.0f);
  ecran.drawFastVLine(x + (int)(ratioOKmax * w), y - 2, h + 4, TFT_WHITE);

  // Repère consigne (jaune) si défini
  if (consigne >= 0) {
    float ratioC = constrain((consigne - valMin) / (valMax - valMin), 0.0f, 1.0f);
    ecran.drawFastVLine(x + (int)(ratioC * w), y - 4, h + 8, TFT_YELLOW);
  }

  // Bordure
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
  ecran.drawString("Capteurs I2C", 120, 20);

  ecran.setTextSize(1);

  // SHT35
  ecran.setTextColor(sht35_ok ? TFT_GREEN : TFT_RED, 0x1E9F);
  ecran.drawString(sht35_ok ? "SHT35 [0x44]  OK"
                            : "SHT35 [0x44]  ABSENT", 120, 70);

  // SCD40
  if (scd40_ok && scd40EnChauffe) {
    ecran.setTextColor(TFT_YELLOW, 0x1E9F);
    ecran.drawString("SCD40 [0x62]  CHAUFFE...", 120, 95);
  } else {
    ecran.setTextColor(scd40_ok ? TFT_GREEN : TFT_RED, 0x1E9F);
    ecran.drawString(scd40_ok ? "SCD40 [0x62]  OK"
                              : "SCD40 [0x62]  ABSENT", 120, 95);
  }

  // Résultat global
  ecran.setTextSize(2);
  bool tousOK = sht35_ok && scd40_ok && !scd40EnChauffe;
  ecran.setTextColor(tousOK ? TFT_GREEN :
                    (sht35_ok || scd40_ok) ? TFT_YELLOW : TFT_RED, 0x1E9F);
  ecran.drawString(tousOK ? "SYSTEME OK" :
                   scd40EnChauffe ? "ATTENTE SCD40" : "ATTENTE...", 120, 145);

  // Timer stabilité ou instruction
  ecran.setTextSize(1);
  ecran.setTextColor(TFT_WHITE, 0x1E9F);
  if (tousOK && tempsCapteursOK > 0) {
    char buf[32];
    uint32_t sec = (millis() - tempsCapteursOK) / 1000;
    snprintf(buf, sizeof(buf), "Stable depuis %lus", sec);
    ecran.drawString(buf, 120, 190);
    ecran.drawString("GO pour continuer", 120, 215);
  } else if (scd40EnChauffe) {
    // Timer chauffe SCD40
    char buf[32];
    uint32_t restant = DUREE_CHAUFFE_SCD40 / 1000 -
                       min((millis() - tempsRedemarrageSCD40) / 1000UL,
                           (unsigned long)DUREE_CHAUFFE_SCD40 / 1000);
    snprintf(buf, sizeof(buf), "SCD40 pret dans %lus", restant);
    ecran.drawString(buf, 120, 190);
  } else {
    ecran.drawString("Brancher SHT35 + SCD40", 120, 190);
    ecran.drawString("GO ignore si capteur absent", 120, 215);
  }
}

// =============================================================
//  AFFICHAGE ÉCRAN TFT — dispatcher
//  240 × 280 px portrait — centre x = 120
// =============================================================
void afficherEcran(int e) {
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
      ecran.drawString("v3.9", 120, 160);
      ecran.drawString("Initialisation...", 120, 200);
      break;

    // --- CHECK ---
    // L'écran est géré par afficherResultatI2C() dans scannerCapteursI2C()
    // On retourne immédiatement pour éviter le flash "Scan I2C..."
    case ETAT_CHECK:
      return;

    // --- BUFFER ---
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

        // Timer stabilité
        if (tempsCapteursOK > 0) {
          uint32_t sec = (millis() - tempsCapteursOK) / 1000;
          snprintf(buf, sizeof(buf), "Stable depuis %lus", sec);
          ecran.setTextColor(TFT_GREEN, 0xFD20);
          ecran.drawString(buf, 120, 135);
        }

        ecran.setTextColor(TFT_BLACK, 0xFD20);
        ecran.drawString("Fermez l'enceinte", 120, 175);
        ecran.drawString("GO pour demarrer", 120, 195);
        break;
      }

    // --- MARCHE ---
    case ETAT_MARCHE:
      {
        ecran.fillScreen(0x07E0);
        ecran.setTextColor(TFT_BLACK, 0x07E0);
        ecran.setTextSize(2);
        ecran.drawString("EN MARCHE", 120, 15);
        ecran.setTextSize(1);
        char ligne[32];

        // Chrono HH:MM:SS
        unsigned long elapsed = (millis() - tempsDebutMarche) / 1000;
        uint8_t hh = elapsed / 3600;
        uint8_t mm = (elapsed % 3600) / 60;
        uint8_t ss = elapsed % 60;
        snprintf(ligne, sizeof(ligne), "%02d:%02d:%02d", hh, mm, ss);
        ecran.drawString(ligne, 120, 35);

        // --- Température ---
        snprintf(ligne, sizeof(ligne), "T: %.1f/%.1f C", temperature, CONSIGNE_TEMP);
        ecran.drawString(ligne, 120, 58);
        // Barre temp : zone OK [36.5 - 37.5], consigne 37.0
        dessinBarre(20, 68, 200, 10,
                    temperature,
                    BARRE_TEMP_MIN, BARRE_TEMP_MAX,
                    SEUIL_CHAUFFE, SEUIL_REFROIDIT,
                    CONSIGNE_TEMP);
        ecran.setTextColor(TFT_BLACK, 0x07E0);
        ecran.drawString("35", 20, 82);
        ecran.drawString("37", 113, 82);
        ecran.drawString("39", 212, 82);

        // --- Humidité ---
        if (alerteHR) {
          ecran.setTextColor(TFT_RED, 0x07E0);
          snprintf(ligne, sizeof(ligne), "HR: %.1f %% ALERTE", humidite);
        } else {
          snprintf(ligne, sizeof(ligne), "HR: %.1f %%", humidite);
        }
        ecran.drawString(ligne, 120, 100);
        // Barre HR : zone OK [85 - 100], consigne 95
        dessinBarre(20, 110, 200, 10,
                    humidite,
                    BARRE_HR_MIN, BARRE_HR_MAX,
                    SEUIL_HR_ALERTE, 100.0f,
                    95.0f);
        ecran.setTextColor(TFT_BLACK, 0x07E0);
        ecran.drawString("0", 20, 124);
        ecran.drawString("50", 107, 124);
        ecran.drawString("100", 205, 124);

        // --- CO₂ ---
        const char* indCO2 = (etatCO2 == CO2_INJECTING)  ? " [INJ]" :
                             (etatCO2 == CO2_STABILIZING) ? " [STB]" : "";
        snprintf(ligne, sizeof(ligne), "CO2: %.2f%%%s", co2, indCO2);
        ecran.drawString(ligne, 120, 142);
        // Barre CO2 : zone OK [4.7 - 5.3], consigne 5.0
        dessinBarre(20, 152, 200, 10,
                    co2,
                    BARRE_CO2_MIN, BARRE_CO2_MAX,
                    SEUIL_BAS_CO2, SEUIL_HAUT_CO2,
                    CONSIGNE_CO2);
        ecran.setTextColor(TFT_BLACK, 0x07E0);
        ecran.drawString("0", 20, 166);
        ecran.drawString("5%", 110, 166);
        ecran.drawString("10%", 203, 166);

        // État Peltier
        const char* labelTherm =
          (etatThermiqueActuel == THERM_CHAUFFE) ? "Chauffe + ventil" :
          (etatThermiqueActuel == THERM_DISSIPE) ? "Dissipe (ventil)" : "Neutre";
        ecran.drawString(labelTherm, 120, 185);

        // Compteurs erreurs si actifs
        if (compteurErreursSHT35 > 0 || compteurErreursSCD40 > 0) {
          ecran.setTextColor(TFT_YELLOW, 0x07E0);
          snprintf(ligne, sizeof(ligne), "Err T:%d CO2:%d",
                   compteurErreursSHT35, compteurErreursSCD40);
          ecran.drawString(ligne, 120, 202);
          ecran.setTextColor(TFT_BLACK, 0x07E0);
        }

        ecran.drawString("GO 2s → pause", 120, 258);
        break;
      }

    // --- PAUSE ---
    case ETAT_PAUSE:
      {
        ecran.fillScreen(0xFC60);
        ecran.setTextColor(TFT_BLACK, 0xFC60);
        ecran.setTextSize(2);
        ecran.drawString("EN PAUSE", 120, 15);
        ecran.setTextSize(1);
        char buf[32];

        // Chrono
        if (tempsDebutMarche > 0) {
          unsigned long elapsed = (millis() - tempsDebutMarche) / 1000;
          uint8_t hh = elapsed / 3600;
          uint8_t mm = (elapsed % 3600) / 60;
          uint8_t ss = elapsed % 60;
          snprintf(buf, sizeof(buf), "%02d:%02d:%02d", hh, mm, ss);
          ecran.drawString(buf, 120, 35);
        }

        ecran.drawString("Actionneurs OFF", 120, 55);

        // Température + barre
        snprintf(buf, sizeof(buf), "T: %.1f C", temperature);
        ecran.drawString(buf, 120, 78);
        dessinBarre(20, 88, 200, 8,
                    temperature,
                    BARRE_TEMP_MIN, BARRE_TEMP_MAX,
                    SEUIL_CHAUFFE, SEUIL_REFROIDIT,
                    CONSIGNE_TEMP);

        // Humidité + barre
        ecran.setTextColor(TFT_BLACK, 0xFC60);
        snprintf(buf, sizeof(buf), "HR: %.1f %%", humidite);
        ecran.drawString(buf, 120, 108);
        dessinBarre(20, 118, 200, 8,
                    humidite,
                    BARRE_HR_MIN, BARRE_HR_MAX,
                    SEUIL_HR_ALERTE, 100.0f,
                    95.0f);

        // CO₂ + barre
        ecran.setTextColor(TFT_BLACK, 0xFC60);
        snprintf(buf, sizeof(buf), "CO2: %.2f %%", co2);
        ecran.drawString(buf, 120, 138);
        dessinBarre(20, 148, 200, 8,
                    co2,
                    BARRE_CO2_MIN, BARRE_CO2_MAX,
                    SEUIL_BAS_CO2, SEUIL_HAUT_CO2,
                    CONSIGNE_CO2);

        ecran.setTextColor(TFT_BLACK, 0xFC60);
        ecran.drawString("GO court  → reprendre", 120, 210);
        ecran.drawString("GO 3s     → retour PRET", 120, 230);
        break;
      }

    // --- ERREUR ---
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
