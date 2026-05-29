// =============================================================
//  PB-0326 — Incubateur transportable pour cellules mammifères
//  HEPIA HES-SO — Groupe 3 : Lyla Founas, Mattéo Pozzo di Borgo
//  Version : 3.7 — Mai 2026
// =============================================================
//
//  MODIFICATIONS v3.7 :
//    - Ajout LED I2C sur GPIO 4 :
//        SHT35 détecté  → LED ON
//        SHT35 absent   → LED OFF
//        Mise à jour à chaque scan et à chaque lecture
//    - Bug ETAT_CHECK_CAPTEURS : scan toutes les 2s (pas juste une fois)
//        SHT35 disparaît après détection → ERREUR immédiate sans GO
//    - Bug ETAT_ATTENTE_DONNEES : NaN → ERREUR immédiate sans GO
//        + suppression code mort après break
//    - toutEteindre() inclut maintenant LED_I2C OFF
//
//  MACHINE À ÉTATS v3.7 :
//    0 - INIT            : splash 3s automatique
//    1 - CHECK_CAPTEURS  : scan I2C toutes les 2s + affichage
//                          SHT35 OK → attente GO → ATTENTE_DONNEES
//                          SHT35 absent au 1er scan → attente GO → ERREUR
//                          SHT35 disparaît après détection → ERREUR immédiate
//    2 - ATTENTE_DONNEES : lecture forcée + affichage + GO pour valider
//                          NaN → ERREUR immédiate
//    3 - BUFFER          : valeurs live + GO pour démarrer
//    4 - MARCHE          : régulation active, GO 2s → PAUSE
//    5 - PAUSE           : actionneurs OFF, GO court → MARCHE, GO 3s → BUFFER
//    6 - ERREUR          : tout éteint, LED clignote, GO 1s → restart
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
#define BROCHE_PELTIER      32  // MOSFET Grove Peltier
#define BROCHE_VENTILATEUR  14  // Ventilateur (Grove MOSFET)
#define BROCHE_SOLENOIDE    27  // Électrovanne V114 — CO₂ DÉSACTIVÉ
#define BROCHE_LED_ROUGE    19  // LED alerte erreur
#define BROCHE_LED_I2C       4  // LED verte capteurs I2C détectés
#define BROCHE_BOUTON_GO     5  // Bouton GO (INPUT_PULLUP)

// =============================================================
//  CONSIGNES BIOLOGIQUES
// =============================================================
#define CONSIGNE_TEMP   37.0f
#define SEUIL_CHAUFFE   36.5f
#define SEUIL_REFROIDIT 37.5f
#define TEMP_SECURITE   40.0f

#define CONSIGNE_CO2   5.0f
#define SEUIL_BAS_CO2  4.7f
#define SEUIL_HAUT_CO2 5.3f

#define SEUIL_HR_ALERTE 85.0f

// =============================================================
//  PARAMÈTRES CO₂ — conservés, non utilisés
// =============================================================
#define DUREE_INJECT_MS     200
#define DUREE_STABILISATION 30000
#define INTERVALLE_CO2      5000

// =============================================================
//  PARAMÈTRES DIVERS
// =============================================================
#define INTERVALLE_ECRAN         2000  // Refresh TFT (ms)
#define INTERVALLE_LECTURE       2000  // Cadence lecture capteurs (ms)
#define INTERVALLE_SCAN_I2C      2000  // Cadence scan I2C en CHECK_CAPTEURS (ms)
#define DUREE_INIT               3000  // Durée INIT (ms)
#define DUREE_ANTIREBOND           50  // Anti-rebond bouton (ms)
#define DUREE_APPUI_PAUSE        2000  // GO maintenu en MARCHE → PAUSE (ms)
#define DUREE_APPUI_BUFFER_PAUSE 3000  // GO maintenu en PAUSE → BUFFER (ms)
#define DUREE_APPUI_RESET        1000  // GO maintenu en ERREUR → restart (ms)
#define TENTATIVES_I2C              3
#define DELAI_RETRY_I2C            50
#define SEUIL_ERREURS_LECTURE       3  // NaN consécutifs → ERREUR en MARCHE

// =============================================================
//  ÉTATS MACHINE PRINCIPALE
// =============================================================
#define ETAT_INIT            0
#define ETAT_CHECK_CAPTEURS  1
#define ETAT_ATTENTE_DONNEES 2
#define ETAT_BUFFER          3
#define ETAT_MARCHE          4
#define ETAT_PAUSE           5
#define ETAT_ERREUR          6

// =============================================================
//  ÉTATS THERMIQUES
// =============================================================
#define THERM_NEUTRE  0
#define THERM_CHAUFFE 1
#define THERM_DISSIPE 2

// =============================================================
//  ÉTATS CO₂ — conservés, inactifs
// =============================================================
#define CO2_IDLE        0
#define CO2_INJECTING   1
#define CO2_STABILIZING 2

// =============================================================
//  ADRESSES I2C
// =============================================================
#define ADRESSE_SHT35 0x44
#define ADRESSE_SCD40 0x62  // DÉSACTIVÉ

// =============================================================
//  COMMANDES SCD40 — DÉSACTIVÉES
// =============================================================
#define SCD40_CMD_START_MEAS 0x21B1
#define SCD40_CMD_READ_MEAS  0xEC05
#define SCD40_CMD_STOP_MEAS  0x3F86

// =============================================================
//  OBJETS
// =============================================================
TFT_eSPI       ecran = TFT_eSPI();
Adafruit_SHT31 sht35 = Adafruit_SHT31();

// =============================================================
//  VARIABLES GLOBALES
// =============================================================

// --- Machine à états ---
int etat          = ETAT_INIT;
int etatPrecedent = -1;

// --- Valeurs capteurs ---
float temperature = 25.0f;
float humidite    = 50.0f;
float co2         =  5.0f;  // Valeur fixe — SCD40 désactivé
                             // ⚠️ Remettre 0.0f quand SCD40 réactivé

// --- Timers ---
unsigned long derniereLecture = 0;
unsigned long dernierEcran    = 0;
unsigned long tempsInit       = 0;
unsigned long dernierScanI2C  = 0;  // Timer scan périodique en CHECK_CAPTEURS

// --- CO₂ ---
int           etatCO2    = CO2_IDLE;
unsigned long timerCO2   = 0;
unsigned long dernierCO2 = 0;

// --- Flags capteurs ---
bool sht35Operationnel = false;
bool co2Actif          = false;  // ⚠️ true pour activer CO₂
bool scd40Demarre      = false;
bool alerteHR          = false;

// --- Scan I2C ---
bool scanEffectue        = false;  // Premier scan effectué dans CHECK_CAPTEURS
bool scanSHT35OK         = false;  // Résultat dernier scan SHT35
bool scanSCD40OK         = false;  // Résultat dernier scan SCD40
bool sht35DetecteUnesFois = false; // true dès que SHT35 détecté au moins une fois
                                   // permet de distinguer "jamais vu" de "perdu"

// --- Flag lecture forcée ATTENTE_DONNEES ---
bool lectureInitialeEffectuee = false;

// --- État thermique mémorisé ---
int etatThermiqueActuel = -1;  // -1 = force premier digitalWrite

// --- Erreur one-shot ---
bool erreurInitialisee = false;

// --- Détection perte capteur en MARCHE ---
uint8_t compteurErreursSHT35 = 0;

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

  // Front descendant : relâché → pressé
  if (lu && !bouton.etatPrecedent) {
    bouton.tempsAppui       = millis();
    bouton.appuiDetecte     = true;
    bouton.appuiCourtValide = false;
  }

  // Front montant : pressé → relâché
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
//  MISE À JOUR LED I2C
//
//  Allumée si SHT35 détecté, éteinte sinon.
//  Appelée après chaque scan et après chaque lecture capteur.
// =============================================================
void mettreAJourLedI2C() {
  digitalWrite(BROCHE_LED_I2C, sht35Operationnel ? HIGH : LOW);
}

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("=== PB-0326 Firmware v3.7 ===");

  pinMode(BROCHE_PELTIER,     OUTPUT);
  pinMode(BROCHE_VENTILATEUR, OUTPUT);
  pinMode(BROCHE_SOLENOIDE,   OUTPUT);
  pinMode(BROCHE_LED_ROUGE,   OUTPUT);
  pinMode(BROCHE_LED_I2C,     OUTPUT);
  pinMode(BROCHE_BOUTON_GO,   INPUT_PULLUP);

  toutEteindre();

  Wire.begin(21, 22);
  delay(100);

  sht35Demarrer();
  mettreAJourLedI2C();

  // SCD40 : DÉSACTIVÉ
  // scd40Demarrer();
  // delay(5000);

  // Écran TFT ST7789V3 240x280
  // User_Setup.h : ST7789_DRIVER, 240x280, MOSI=23, SCLK=18, CS=15, DC=2, RST=33
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

  // Lecture capteurs cadencée toutes les 2s
  if (millis() - derniereLecture >= INTERVALLE_LECTURE) {
    lireCapteurs();
    derniereLecture = millis();
  }

  // Refresh TFT + reset flags au changement d'état
  if (etat != etatPrecedent) {
    afficherEcran(etat);
    etatPrecedent = etat;
    if (etat != ETAT_CHECK_CAPTEURS) {
      scanEffectue         = false;
      sht35DetecteUnesFois = false;
    }
    if (etat != ETAT_ATTENTE_DONNEES) lectureInitialeEffectuee = false;
    if (etat != ETAT_ERREUR)          erreurInitialisee        = false;
    Serial.printf("[ÉTAT] → %d\n", etat);
  }

  switch (etat) {

    // ----------------------------------------------------------
    //  ÉTAT 0 — INIT
    // ----------------------------------------------------------
    case ETAT_INIT:
      if (millis() - tempsInit >= DUREE_INIT) {
        etat = ETAT_CHECK_CAPTEURS;
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 1 — CHECK CAPTEURS
    //
    //  Scan I2C toutes les 2s, résultat affiché en continu.
    //
    //  Comportement selon résultat :
    //    Jamais détecté : affiche ABSENT + instruction GO → ERREUR
    //    Détecté         : affiche OK + instruction GO → ATTENTE_DONNEES
    //    Détecté puis perdu (entre deux scans) : ERREUR immédiate sans GO
    //      → empêche l'opérateur de continuer avec un capteur instable
    // ----------------------------------------------------------
    case ETAT_CHECK_CAPTEURS:

      // Scan périodique toutes les 2s
      if (!scanEffectue ||
          (millis() - dernierScanI2C >= INTERVALLE_SCAN_I2C)) {
        verifierCapteursI2C();
        dernierScanI2C = millis();

        if (!scanEffectue) {
          scanEffectue = true;
          Serial.println("[CHECK] Premier scan effectué");
        }

        // SHT35 détecté pour la première fois
        if (scanSHT35OK && !sht35DetecteUnesFois) {
          sht35DetecteUnesFois = true;
          Serial.println("[CHECK] SHT35 détecté pour la première fois");
        }

        // SHT35 était détecté et a disparu → ERREUR immédiate
        if (!scanSHT35OK && sht35DetecteUnesFois) {
          Serial.println("[CHECK] SHT35 perdu → ERREUR immédiate");
          signalerErreur("SHT35 perdu");
          break;
        }

        Serial.println("[CHECK] GO pour continuer");
      }

      // Attente GO — décision selon dernier résultat scan
      if (boutonAppuiCourt()) {
        if (scanSHT35OK) {
          etat = ETAT_ATTENTE_DONNEES;
        } else {
          signalerErreur("SHT35 requis");
        }
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 2 — ATTENTE DONNÉES
    //
    //  Lecture forcée à l'entrée (valeurs mises à NAN avant lecture).
    //  Rafraîchissement toutes les 2s.
    //  NaN détecté → ERREUR immédiate (capteur instable ou débranché).
    //  GO → BUFFER si valeurs valides.
    //  GO → ERREUR si valeurs invalides.
    // ----------------------------------------------------------
    case ETAT_ATTENTE_DONNEES:

      // Lecture forcée à l'entrée
      if (!lectureInitialeEffectuee) {
        temperature = NAN;
        humidite    = NAN;
        lireCapteurs();
        derniereLecture          = millis();
        lectureInitialeEffectuee = true;
        afficherEcran(ETAT_ATTENTE_DONNEES);
        Serial.printf("[DONNÉES] T=%.1f HR=%.1f — GO pour valider\n",
                      temperature, humidite);
      }

      // NaN en ATTENTE_DONNEES → ERREUR immédiate
      // (capteur répondait au scan I2C mais ne donne pas de données valides)
      if (isnan(temperature) || isnan(humidite)) {
        Serial.println("[DONNÉES] NaN détecté → ERREUR immédiate");
        signalerErreur("SHT35 NaN");
        break;
      }

      // Rafraîchissement toutes les 2s
      if (millis() - dernierEcran >= INTERVALLE_ECRAN) {
        lireCapteurs();
        derniereLecture = millis();
        afficherEcran(ETAT_ATTENTE_DONNEES);
        dernierEcran = millis();
      }

      // GO → validation manuelle
      if (boutonAppuiCourt()) {
        if (!isnan(temperature) && !isnan(humidite) &&
            temperature > 10.0f  && temperature < 45.0f &&
            humidite    > 0.0f) {
          Serial.println("[DONNÉES] OK → BUFFER");
          etat = ETAT_BUFFER;
        } else {
          Serial.printf("[DONNÉES] Invalides T=%.1f HR=%.1f → ERREUR\n",
                        temperature, humidite);
          signalerErreur("Donnees invalides");
        }
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 3 — BUFFER
    // ----------------------------------------------------------
    case ETAT_BUFFER:
      if (millis() - dernierEcran >= INTERVALLE_ECRAN) {
        afficherEcran(ETAT_BUFFER);
        dernierEcran = millis();
      }

      if (boutonAppuiCourt()) {
        Serial.println("[BUFFER] GO → MARCHE");
        dernierCO2           = millis();
        dernierEcran         = millis();
        etatCO2              = CO2_IDLE;
        compteurErreursSHT35 = 0;
        etatThermiqueActuel  = -1;
        etat                 = ETAT_MARCHE;
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 4 — MARCHE
    // ----------------------------------------------------------
    case ETAT_MARCHE:
      if (temperature >= TEMP_SECURITE) {
        signalerErreur("TEMP CRITIQUE");
        break;
      }

      regulerTemperature();
      miseAJourCO2();
      miseAJourHumidite();

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
    //  ÉTAT 5 — PAUSE
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

      // Appui long 3s → BUFFER (prioritaire)
      if (boutonAppuiLong(DUREE_APPUI_BUFFER_PAUSE)) {
        Serial.println("[PAUSE] GO 3s → BUFFER");
        bouton.appuiCourtValide = false;
        etatThermiqueActuel     = -1;
        compteurErreursSHT35    = 0;
        dernierEcran            = millis();
        etat                    = ETAT_BUFFER;

      // Appui court → MARCHE
      } else if (boutonAppuiCourt()) {
        Serial.println("[PAUSE] GO → MARCHE");
        etatThermiqueActuel  = -1;
        compteurErreursSHT35 = 0;
        dernierEcran         = millis();
        etat                 = ETAT_MARCHE;
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 6 — ERREUR
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
//  SHT35 — Démarrage
// =============================================================
void sht35Demarrer() {
  if (sht35.begin(ADRESSE_SHT35)) {
    sht35Operationnel = true;
    Serial.println("[SHT35] OK");
  } else {
    sht35Operationnel = false;
    Serial.println("[SHT35] ABSENT");
  }
}

// =============================================================
//  SCD40 — DÉSACTIVÉ
//
//  Pour réactiver :
//    1. Décommenter corps scd40EnvoyerCommande() et scd40Demarrer()
//    2. Décommenter scd40Demarrer() + delay(5000) dans setup()
//    3. co2Actif = true
//    4. co2 = 0.0f dans variables globales
//    5. Décommenter blocs SCD40 dans verifierCapteursI2C() et lireCapteurs()
// =============================================================
void scd40EnvoyerCommande(uint16_t cmd) {
  // ⚠️ DÉSACTIVÉ
  // Wire.beginTransmission(ADRESSE_SCD40);
  // Wire.write((uint8_t)(cmd >> 8));
  // Wire.write((uint8_t)(cmd & 0xFF));
  // Wire.endTransmission();
}

void scd40Demarrer() {
  // ⚠️ DÉSACTIVÉ
  // scd40EnvoyerCommande(SCD40_CMD_START_MEAS);
  // scd40Demarre = true;
}

// =============================================================
//  SCD40 — CRC-8 — CONSERVÉ
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
//  SCD40 — Lecture — DÉSACTIVÉE
// =============================================================
bool scd40Lire(float& co2_out, float& temp_out, float& hum_out) {
  // ⚠️ DÉSACTIVÉ
  // scd40EnvoyerCommande(SCD40_CMD_READ_MEAS);
  // delay(1);
  // Wire.requestFrom((uint8_t)ADRESSE_SCD40, (uint8_t)9);
  // if (Wire.available() < 9) return false;
  // uint8_t data[9];
  // for (uint8_t i = 0; i < 9; i++) { data[i] = Wire.read(); }
  // if (calculerCRC(data,     2) != data[2]) return false;
  // if (calculerCRC(data + 3, 2) != data[5]) return false;
  // if (calculerCRC(data + 6, 2) != data[8]) return false;
  // uint16_t raw_co2  = ((uint16_t)data[0] << 8) | data[1];
  // uint16_t raw_temp = ((uint16_t)data[3] << 8) | data[4];
  // uint16_t raw_hum  = ((uint16_t)data[6] << 8) | data[7];
  // co2_out  = (float)raw_co2;
  // temp_out = -45.0f + 175.0f * (raw_temp / 65535.0f);
  // hum_out  = 100.0f * (raw_hum  / 65535.0f);
  // return true;
  return false;
}

// =============================================================
//  LECTURE CAPTEURS — cadencée toutes les 2s
//
//  Détection perte capteur à chaud :
//    NaN consécutifs → compteurErreursSHT35++
//    Lecture valide  → compteur = 0 + LED I2C ON
//    3 NaN en MARCHE → signalerErreur (6s de tolérance)
//    En PAUSE        → NaN comptés mais pas de ERREUR
// =============================================================
void lireCapteurs() {
  if (sht35Operationnel) {
    float t = sht35.readTemperature();
    float h = sht35.readHumidity();

    if (!isnan(t) && !isnan(h)) {
      temperature          = t;
      humidite             = h;
      compteurErreursSHT35 = 0;
      mettreAJourLedI2C();  // LED ON — capteur répond
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

  // ⚠️ DÉSACTIVÉ — SCD40
  // if (scd40Demarre) {
  //   float co2_ppm, temp_scd, hum_scd;
  //   if (scd40Lire(co2_ppm, temp_scd, hum_scd)) {
  //     co2 = co2_ppm / 10000.0f;
  //   }
  // }
}

// =============================================================
//  SCAN I2C
// =============================================================
bool scannerAdresse(uint8_t adresse, uint8_t tentatives) {
  for (uint8_t i = 0; i < tentatives; i++) {
    Wire.beginTransmission(adresse);
    uint8_t erreur = Wire.endTransmission();
    if (erreur == 0) return true;
    Serial.printf("[I2C] %d/%d 0x%02X err=%d\n", i+1, tentatives, adresse, erreur);
    if (i < tentatives - 1) delay(DELAI_RETRY_I2C);
  }
  return false;
}

// =============================================================
//  VÉRIFICATION CAPTEURS I2C
//  Met à jour scanSHT35OK, sht35Operationnel, LED I2C
// =============================================================
void verifierCapteursI2C() {
  Serial.println("[I2C] Scan...");

  scanSHT35OK       = scannerAdresse(ADRESSE_SHT35, TENTATIVES_I2C);
  sht35Operationnel = scanSHT35OK;
  mettreAJourLedI2C();  // LED ON si détecté, OFF si absent
  Serial.printf("[I2C] SHT35 → %s\n", scanSHT35OK ? "OK" : "ABSENT");

  // ⚠️ DÉSACTIVÉ — SCD40
  // scanSCD40OK = scannerAdresse(ADRESSE_SCD40, TENTATIVES_I2C);
  // if (scanSCD40OK) co2Actif = true;
  scanSCD40OK = false;

  afficherResultatI2C(scanSHT35OK, scanSCD40OK);
}

// =============================================================
//  RÉGULATION THERMIQUE — état mémorisé
//
//  digitalWrite émis UNIQUEMENT au changement d'état thermique.
//
//  T < 36.5°C  → THERM_CHAUFFE : Peltier ON  + Ventilateur ON
//  T > 37.5°C  → THERM_DISSIPE : Peltier OFF + Ventilateur ON
//  Zone neutre → THERM_NEUTRE  : tout OFF
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
      Serial.printf("[THERM] → Chauffe  T=%.1fC\n", temperature);
      break;
    case THERM_DISSIPE:
      digitalWrite(BROCHE_PELTIER,     LOW);
      digitalWrite(BROCHE_VENTILATEUR, HIGH);
      Serial.printf("[THERM] → Dissipe  T=%.1fC\n", temperature);
      break;
    case THERM_NEUTRE:
      digitalWrite(BROCHE_PELTIER,     LOW);
      digitalWrite(BROCHE_VENTILATEUR, LOW);
      Serial.printf("[THERM] → Neutre   T=%.1fC\n", temperature);
      break;
  }
}

// =============================================================
//  RÉGULATION CO₂ — DÉSACTIVÉE
// =============================================================
void miseAJourCO2() {
  if (!co2Actif) return;  // ⚠️ DÉSACTIVÉ

  unsigned long maintenant = millis();
  switch (etatCO2) {
    case CO2_IDLE:
      if (maintenant - dernierCO2 >= INTERVALLE_CO2) {
        dernierCO2 = maintenant;
        if (co2 < SEUIL_BAS_CO2) {
          digitalWrite(BROCHE_SOLENOIDE, HIGH);
          timerCO2 = maintenant;
          etatCO2  = CO2_INJECTING;
          Serial.printf("[CO2] Injection (%.2f%%)\n", co2);
        }
      }
      break;
    case CO2_INJECTING:
      if (maintenant - timerCO2 >= DUREE_INJECT_MS) {
        digitalWrite(BROCHE_SOLENOIDE, LOW);
        timerCO2 = maintenant;
        etatCO2  = CO2_STABILIZING;
        Serial.println("[CO2] Vanne fermée → stabilisation");
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
//  MONITORING HUMIDITÉ
// =============================================================
void miseAJourHumidite() {
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
}

// =============================================================
//  TOUT ÉTEINDRE
// =============================================================
void toutEteindre() {
  digitalWrite(BROCHE_PELTIER,     LOW);
  digitalWrite(BROCHE_VENTILATEUR, LOW);
  digitalWrite(BROCHE_SOLENOIDE,   LOW);
  digitalWrite(BROCHE_LED_ROUGE,   LOW);
  digitalWrite(BROCHE_LED_I2C,     LOW);  // LED I2C OFF en erreur
  etatThermiqueActuel = -1;
}

// =============================================================
//  SIGNALER ERREUR
// =============================================================
void signalerErreur(const char* message) {
  Serial.printf("[ERREUR] %s\n", message);
  toutEteindre();
  compteurErreursSHT35 = 0;
  erreurInitialisee    = false;
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
  ecran.drawString("Capteurs I2C", 120, 25);

  ecran.setTextSize(1);
  ecran.setTextColor(sht35_ok ? TFT_GREEN : TFT_RED, 0x1E9F);
  ecran.drawString(sht35_ok ? "SHT35 [0x44]  OK"
                            : "SHT35 [0x44]  ABSENT", 120, 80);

  ecran.setTextColor(0xC618, 0x1E9F);
  ecran.drawString("SCD40 [0x62]  DESACTIVE", 120, 105);

  ecran.setTextSize(2);
  ecran.setTextColor(sht35_ok ? TFT_GREEN : TFT_RED, 0x1E9F);
  ecran.drawString(sht35_ok ? "SHT35 OK" : "SHT35 ABSENT", 120, 155);

  ecran.setTextSize(1);
  ecran.setTextColor(TFT_WHITE, 0x1E9F);
  ecran.drawString(sht35_ok ? "GO pour continuer"
                            : "GO → ERREUR", 120, 210);
}

// =============================================================
//  AFFICHAGE ÉCRAN TFT — dispatcher
//  240 × 280 px portrait — centre x = 120
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
      ecran.drawString("Incubateur transportable", 120, 110);
      ecran.drawString("HEPIA HES-SO  Groupe 3", 120, 135);
      ecran.drawString("Initialisation...", 120, 185);
      break;

    case ETAT_CHECK_CAPTEURS:
      // Provisoire — remplacé par afficherResultatI2C() après scan
      ecran.fillScreen(0x1E9F);
      ecran.setTextColor(TFT_WHITE, 0x1E9F);
      ecran.setTextSize(2);
      ecran.drawString("Scan I2C...", 120, 130);
      break;

    case ETAT_ATTENTE_DONNEES:
      {
        ecran.fillScreen(0x1E9F);
        ecran.setTextColor(TFT_WHITE, 0x1E9F);
        ecran.setTextSize(2);
        ecran.drawString("Verification", 120, 30);
        ecran.setTextSize(1);

        char buf[32];
        snprintf(buf, sizeof(buf), "T:  %.1f C", temperature);
        ecran.drawString(buf, 120, 100);

        snprintf(buf, sizeof(buf), "HR: %.1f %%", humidite);
        ecran.drawString(buf, 120, 125);

        bool valide = (!isnan(temperature) && !isnan(humidite) &&
                       temperature > 10.0f  && temperature < 45.0f &&
                       humidite    > 0.0f);
        ecran.setTextColor(valide ? TFT_GREEN : TFT_RED, 0x1E9F);
        ecran.drawString(valide ? "Valeurs OK" : "Valeurs INVALIDES", 120, 165);

        ecran.setTextColor(TFT_WHITE, 0x1E9F);
        ecran.drawString(valide ? "GO pour continuer"
                                : "GO → ERREUR", 120, 215);
        ecran.drawString("(verifier capteur si invalide)", 120, 235);
        break;
      }

    case ETAT_BUFFER:
      {
        ecran.fillScreen(0xFD20);
        ecran.setTextColor(TFT_BLACK, 0xFD20);
        ecran.setTextSize(2);
        ecran.drawString("PRET", 120, 35);
        ecran.setTextSize(1);
        char buf[32];

        snprintf(buf, sizeof(buf), "T:   %.1f C", temperature);
        ecran.drawString(buf, 120, 95);

        snprintf(buf, sizeof(buf), "HR:  %.1f %%", humidite);
        ecran.drawString(buf, 120, 120);

        ecran.setTextColor(0x7BEF, 0xFD20);
        snprintf(buf, sizeof(buf), "CO2: %.2f %% [OFF]", co2);
        ecran.drawString(buf, 120, 145);

        ecran.setTextColor(TFT_BLACK, 0xFD20);
        ecran.drawString("Fermez l'enceinte", 120, 190);
        ecran.drawString("GO pour demarrer", 120, 215);
        break;
      }

    case ETAT_MARCHE:
      {
        ecran.fillScreen(0x07E0);
        ecran.setTextColor(TFT_BLACK, 0x07E0);
        ecran.setTextSize(2);
        ecran.drawString("EN MARCHE", 120, 25);
        ecran.setTextSize(1);
        char ligne[32];

        const char* indT =
          (etatThermiqueActuel == THERM_CHAUFFE) ? " [+]" :
          (etatThermiqueActuel == THERM_DISSIPE) ? " [-]" : " [=]";
        snprintf(ligne, sizeof(ligne), "T: %.1f/%.1f C%s",
                 temperature, CONSIGNE_TEMP, indT);
        ecran.drawString(ligne, 120, 80);

        if (alerteHR) {
          ecran.setTextColor(TFT_RED, 0x07E0);
          snprintf(ligne, sizeof(ligne), "HR: %.1f %% ALERTE", humidite);
        } else {
          snprintf(ligne, sizeof(ligne), "HR: %.1f %%", humidite);
        }
        ecran.drawString(ligne, 120, 108);
        ecran.setTextColor(TFT_BLACK, 0x07E0);

        ecran.setTextColor(0x39E7, 0x07E0);
        snprintf(ligne, sizeof(ligne), "CO2: %.2f %% [OFF]", co2);
        ecran.drawString(ligne, 120, 136);
        ecran.setTextColor(TFT_BLACK, 0x07E0);

        const char* labelTherm =
          (etatThermiqueActuel == THERM_CHAUFFE) ? "Chauffe + ventil" :
          (etatThermiqueActuel == THERM_DISSIPE) ? "Dissipe (ventil)" : "Neutre";
        ecran.drawString(labelTherm, 120, 164);

        if (compteurErreursSHT35 > 0) {
          ecran.setTextColor(TFT_YELLOW, 0x07E0);
          snprintf(ligne, sizeof(ligne), "SHT35 err: %d/%d",
                   compteurErreursSHT35, SEUIL_ERREURS_LECTURE);
          ecran.drawString(ligne, 120, 192);
          ecran.setTextColor(TFT_BLACK, 0x07E0);
        }

        ecran.drawString("GO 2s pour pause", 120, 255);
        break;
      }

    case ETAT_PAUSE:
      {
        ecran.fillScreen(0xFC60);
        ecran.setTextColor(TFT_BLACK, 0xFC60);
        ecran.setTextSize(2);
        ecran.drawString("EN PAUSE", 120, 25);
        ecran.setTextSize(1);
        char buf[32];

        ecran.drawString("Regulation suspendue", 120, 65);
        ecran.drawString("Actionneurs OFF", 120, 85);

        snprintf(buf, sizeof(buf), "T:   %.1f C", temperature);
        ecran.drawString(buf, 120, 125);

        snprintf(buf, sizeof(buf), "HR:  %.1f %%", humidite);
        ecran.drawString(buf, 120, 150);

        ecran.setTextColor(0x7BEF, 0xFC60);
        snprintf(buf, sizeof(buf), "CO2: %.2f %% [OFF]", co2);
        ecran.drawString(buf, 120, 175);

        ecran.setTextColor(TFT_BLACK, 0xFC60);
        ecran.drawString("GO court  → reprendre", 120, 215);
        ecran.drawString("GO 3s     → retour PRET", 120, 235);
        break;
      }

    case ETAT_ERREUR:
      ecran.fillScreen(TFT_RED);
      ecran.setTextColor(TFT_WHITE, TFT_RED);
      ecran.setTextSize(2);
      ecran.drawString("ERREUR", 120, 70);
      ecran.setTextSize(1);
      ecran.drawString("Systeme arrete", 120, 130);
      ecran.drawString("Actionneurs OFF", 120, 155);
      ecran.drawString("GO 1s → redemarrer", 120, 200);
      break;
  }
}
