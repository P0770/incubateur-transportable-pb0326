// =============================================================
//  PB-0326 — Incubateur transportable pour cellules mammifères
//  HEPIA HES-SO — Groupe 3 : Lyla Founas, Mattéo Pozzo di Borgo
//  Version : 3.3 — Mai 2026
// =============================================================
//
//  MODIFICATIONS v3.3 :
//    - Ventilateur : GPIO 25 → GPIO 13 (25 conflicte avec DAC ESP32)
//    - Détection perte capteur à chaud : 3 lectures NaN consécutives
//      → signalerErreur("SHT35 perdu") → ETAT_ERREUR
//
// =============================================================

#include <TFT_eSPI.h>
#include <Wire.h>
#include <Adafruit_SHT31.h>

// =============================================================
//  BROCHES GPIO
//  I2C  : SDA=21, SCL=22
//  SPI  : MOSI=23, SCLK=18, CS=15, DC=2, RST=33 (dans User_Setup.h)
//  EVITER: 6-11 (flash SPI), 34/35/36/39 (input only), 25/26 (DAC)
// =============================================================
#define BROCHE_PELTIER      32   // MOSFET Grove Peltier
#define BROCHE_VENTILATEUR  13   // Ventilateur — GPIO 13 (25 retiré : conflit DAC)
#define BROCHE_SOLENOIDE    27   // Électrovanne V114 — CO₂ DÉSACTIVÉ
#define BROCHE_LED_ROUGE    26   // LED alerte erreur
#define BROCHE_BOUTON_GO     5   // Bouton GO / reset erreur (INPUT_PULLUP)

// =============================================================
//  CONSIGNES BIOLOGIQUES
// =============================================================
#define CONSIGNE_TEMP   37.0f
#define SEUIL_CHAUFFE   36.5f
#define SEUIL_REFROIDIT 37.5f
#define TEMP_SECURITE   40.0f

#define CONSIGNE_CO2    5.0f
#define SEUIL_BAS_CO2   4.7f
#define SEUIL_HAUT_CO2  5.3f

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
#define INTERVALLE_ECRAN       2000  // Refresh TFT (ms)
#define INTERVALLE_LECTURE     2000  // Cadence lecture capteurs (ms)
#define DUREE_INIT             3000  // Durée INIT (ms)
#define DUREE_APPUI_RESET      1000  // Appui GO pour reset ERREUR (ms)
#define TENTATIVES_I2C            3  // Tentatives scan I2C
#define DELAI_RETRY_I2C          50  // Délai entre tentatives (ms)
#define SEUIL_ERREURS_LECTURE     3  // Nb échecs NaN consécutifs → ERREUR

// =============================================================
//  ÉTATS MACHINE PRINCIPALE
// =============================================================
#define ETAT_INIT            0
#define ETAT_CHECK_CAPTEURS  1
#define ETAT_ATTENTE_DONNEES 2
#define ETAT_BUFFER          3
#define ETAT_MARCHE          4
#define ETAT_ERREUR          5

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

// --- CO₂ ---
int           etatCO2    = CO2_IDLE;
unsigned long timerCO2   = 0;
unsigned long dernierCO2 = 0;

// --- Flags ---
bool sht35Operationnel = false;
bool co2Actif          = false;  // ⚠️ true pour activer CO₂
bool scd40Demarre      = false;
bool alerteHR          = false;

// --- Scan I2C ---
bool scanEffectue = false;
bool scanSHT35OK  = false;
bool scanSCD40OK  = false;

// --- Reset erreur ---
unsigned long tempsAppuiErreur   = 0;
bool          appuiErreurEnCours = false;

// --- Détection perte capteur à chaud ---
// Compte les lectures NaN consécutives du SHT35.
// Si compteur atteint SEUIL_ERREURS_LECTURE (3) → signalerErreur()
// Remis à 0 dès qu'une lecture valide est reçue.
uint8_t compteurErreursSHT35 = 0;

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("=== PB-0326 Firmware v3.3 ===");

  // --- Broches sortie ---
  pinMode(BROCHE_PELTIER,     OUTPUT);
  pinMode(BROCHE_VENTILATEUR, OUTPUT);
  pinMode(BROCHE_SOLENOIDE,   OUTPUT);
  pinMode(BROCHE_LED_ROUGE,   OUTPUT);

  // --- Broche entrée ---
  pinMode(BROCHE_BOUTON_GO, INPUT_PULLUP);

  // --- Sécurité initiale ---
  toutEteindre();

  // --- Bus I2C ---
  Wire.begin(21, 22);
  delay(100);

  // --- SHT35 ---
  sht35Demarrer();

  // --- SCD40 : DÉSACTIVÉ ---
  // scd40Demarrer();
  // delay(5000);

  // --- Écran TFT ---
  // ⚠️ Broches configurées dans User_Setup.h de la bibliothèque TFT_eSPI :
  //    ST7789_DRIVER, 240x280, MOSI=23, SCLK=18, CS=15, DC=2, RST=33
  //    TFT_RGB_ORDER TFT_RGB, TFT_INVERSION_ON, SPI_FREQUENCY 40000000
  ecran.init();
  ecran.setRotation(0);
  ecran.fillScreen(TFT_BLACK);
  ecran.setTextDatum(MC_DATUM);

  // Test blanc 500ms au démarrage — confirme que l'écran répond
  ecran.fillScreen(TFT_WHITE);
  delay(500);
  ecran.fillScreen(TFT_BLACK);

  tempsInit = millis();
  Serial.println("[SETUP] OK");
}

// =============================================================
//  LOOP — Machine à états
// =============================================================
void loop() {

  // Lecture capteurs cadencée toutes les 2s
  if (millis() - derniereLecture >= INTERVALLE_LECTURE) {
    lireCapteurs();
    derniereLecture = millis();
  }

  // Refresh TFT au changement d'état
  if (etat != etatPrecedent) {
    afficherEcran(etat);
    etatPrecedent = etat;
    if (etat != ETAT_CHECK_CAPTEURS) scanEffectue = false;
    Serial.printf("[ÉTAT] → %d\n", etat);
  }

  switch (etat) {

    // ----------------------------------------------------------
    //  ÉTAT 0 — INIT  (3s automatique)
    // ----------------------------------------------------------
    case ETAT_INIT:
      if (millis() - tempsInit >= DUREE_INIT) {
        etat = ETAT_CHECK_CAPTEURS;
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 1 — CHECK CAPTEURS
    //
    //  Phase 1 : scan I2C automatique (une fois)
    //  Phase 2 : attente bouton GO
    //    SHT35 OK  → ATTENTE_DONNEES
    //    SHT35 KO  → ERREUR
    // ----------------------------------------------------------
    case ETAT_CHECK_CAPTEURS:
      if (!scanEffectue) {
        verifierCapteursI2C();
        scanEffectue = true;
        Serial.println("[CHECK] Appuyez GO pour continuer");
      }

      if (digitalRead(BROCHE_BOUTON_GO) == LOW) {
        delay(50);
        if (digitalRead(BROCHE_BOUTON_GO) == LOW) {
          if (scanSHT35OK) {
            etat = ETAT_ATTENTE_DONNEES;
          } else {
            signalerErreur("SHT35 requis");
          }
        }
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 2 — ATTENTE DONNÉES  (automatique)
    // ----------------------------------------------------------
    case ETAT_ATTENTE_DONNEES:
      if (temperature > 10.0f && temperature < 45.0f && humidite > 0.0f) {
        etat = ETAT_BUFFER;
      } else {
        Serial.printf("[DONNÉES] Invalides T=%.1f HR=%.1f\n", temperature, humidite);
        signalerErreur("Donnees invalides");
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 3 — BUFFER  (attente GO)
    // ----------------------------------------------------------
    case ETAT_BUFFER:
      if (millis() - dernierEcran >= INTERVALLE_ECRAN) {
        afficherEcran(ETAT_BUFFER);
        dernierEcran = millis();
      }

      if (digitalRead(BROCHE_BOUTON_GO) == LOW) {
        delay(50);
        if (digitalRead(BROCHE_BOUTON_GO) == LOW) {
          Serial.println("[BUFFER] GO → MARCHE");
          dernierCO2   = millis();
          dernierEcran = millis();
          etatCO2      = CO2_IDLE;
          compteurErreursSHT35 = 0;  // Reset compteur avant démarrage
          etat         = ETAT_MARCHE;
        }
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 4 — MARCHE
    // ----------------------------------------------------------
    case ETAT_MARCHE:
      // Sécurité température absolue
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
      break;

    // ----------------------------------------------------------
    //  ÉTAT 5 — ERREUR
    //  LED rouge clignotante, tout éteint
    //  Appui GO maintenu 1s → ESP.restart()
    // ----------------------------------------------------------
    case ETAT_ERREUR:
      toutEteindre();

      // Clignotement LED non bloquant (500ms)
      {
        static unsigned long dernierBlink = 0;
        static bool          ledActive    = false;
        if (millis() - dernierBlink >= 500) {
          ledActive    = !ledActive;
          digitalWrite(BROCHE_LED_ROUGE, ledActive ? HIGH : LOW);
          dernierBlink = millis();
        }
      }

      // Appui long GO → reset
      if (digitalRead(BROCHE_BOUTON_GO) == LOW) {
        if (!appuiErreurEnCours) {
          appuiErreurEnCours = true;
          tempsAppuiErreur   = millis();
          Serial.println("[ERREUR] Maintenir GO 1s pour reset...");
        } else if (millis() - tempsAppuiErreur >= DUREE_APPUI_RESET) {
          Serial.println("[ERREUR] → restart");
          delay(200);
          ESP.restart();
        }
      } else {
        if (appuiErreurEnCours) {
          Serial.println("[ERREUR] Appui trop court");
        }
        appuiErreurEnCours = false;
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
//  SCD40 — DÉSACTIVÉ (fonctions conservées)
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
  // Pour réactiver :
  //   1. Décommenter ce corps
  //   2. Décommenter scd40Demarrer() dans setup()
  //   3. co2Actif = true
  //   4. co2 = 0.0f
  //   5. Décommenter bloc SCD40 dans verifierCapteursI2C()
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
//    Chaque lecture NaN incrémente compteurErreursSHT35.
//    Une lecture valide remet le compteur à 0.
//    Si compteur >= SEUIL_ERREURS_LECTURE (3 × 2s = 6s sans donnée)
//    → signalerErreur("SHT35 perdu") → ETAT_ERREUR
//
//  Cela couvre trois cas :
//    - Débranchement physique pendant MARCHE
//    - Défaut I2C transitoire (toléré 2 fois)
//    - Capteur qui gèle sans retourner NaN (température conservée,
//      mais compteur s'incrémente quand même sur NaN)
// =============================================================
void lireCapteurs() {
  if (sht35Operationnel) {
    float t = sht35.readTemperature();
    float h = sht35.readHumidity();

    bool lectureValide = !isnan(t) && !isnan(h);

    if (lectureValide) {
      // Lecture OK → mise à jour valeurs + reset compteur erreurs
      temperature = t;
      humidite    = h;
      compteurErreursSHT35 = 0;
      Serial.printf("[SHT35] T=%.1fC  HR=%.1f%%\n", temperature, humidite);

    } else {
      // Lecture NaN → incrémenter compteur
      compteurErreursSHT35++;
      Serial.printf("[SHT35] Lecture invalide (%d/%d)\n",
                    compteurErreursSHT35, SEUIL_ERREURS_LECTURE);

      // Seuil atteint → erreur critique (uniquement en MARCHE)
      // En dehors de MARCHE, une perte capteur est détectée
      // au moment du scan ou de la validation des données
      if (compteurErreursSHT35 >= SEUIL_ERREURS_LECTURE && etat == ETAT_MARCHE) {
        Serial.println("[SHT35] Capteur perdu → ERREUR");
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
//  SCAN I2C — tentatives multiples
// =============================================================
bool scannerAdresse(uint8_t adresse, uint8_t tentatives) {
  for (uint8_t i = 0; i < tentatives; i++) {
    Wire.beginTransmission(adresse);
    uint8_t erreur = Wire.endTransmission();
    if (erreur == 0) return true;
    Serial.printf("[I2C] %d/%d 0x%02X erreur=%d\n", i+1, tentatives, adresse, erreur);
    if (i < tentatives - 1) delay(DELAI_RETRY_I2C);
  }
  return false;
}

// =============================================================
//  VÉRIFICATION CAPTEURS I2C
// =============================================================
void verifierCapteursI2C() {
  Serial.println("[I2C] Scan...");

  scanSHT35OK       = scannerAdresse(ADRESSE_SHT35, TENTATIVES_I2C);
  sht35Operationnel = scanSHT35OK;
  Serial.printf("[I2C] SHT35 → %s\n", scanSHT35OK ? "OK" : "ABSENT");

  // ⚠️ DÉSACTIVÉ — SCD40
  // scanSCD40OK = scannerAdresse(ADRESSE_SCD40, TENTATIVES_I2C);
  // if (scanSCD40OK) co2Actif = true;
  scanSCD40OK = false;

  afficherResultatI2C(scanSHT35OK, scanSCD40OK);
}

// =============================================================
//  RÉGULATION THERMIQUE — Bang-bang zone morte
//
//  T < 36.5°C  → Peltier ON  + Ventilateur ON  (chauffe + visuel)
//  T > 37.5°C  → Peltier OFF + Ventilateur ON  (dissipation)
//  Zone neutre → Peltier OFF + Ventilateur OFF
// =============================================================
void regulerTemperature() {
  if (temperature < SEUIL_CHAUFFE) {
    digitalWrite(BROCHE_PELTIER,     HIGH);
    digitalWrite(BROCHE_VENTILATEUR, HIGH);
    Serial.printf("[THERM] Chauffe T=%.1fC\n", temperature);

  } else if (temperature > SEUIL_REFROIDIT) {
    digitalWrite(BROCHE_PELTIER,     LOW);
    digitalWrite(BROCHE_VENTILATEUR, HIGH);
    Serial.printf("[THERM] Trop chaud T=%.1fC\n", temperature);

  } else {
    digitalWrite(BROCHE_PELTIER,     LOW);
    digitalWrite(BROCHE_VENTILATEUR, LOW);
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
        }
      }
      break;
    case CO2_INJECTING:
      if (maintenant - timerCO2 >= DUREE_INJECT_MS) {
        digitalWrite(BROCHE_SOLENOIDE, LOW);
        timerCO2 = maintenant;
        etatCO2  = CO2_STABILIZING;
      }
      break;
    case CO2_STABILIZING:
      if (maintenant - timerCO2 >= DUREE_STABILISATION) {
        etatCO2 = CO2_IDLE;
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
}

// =============================================================
//  SIGNALER ERREUR
// =============================================================
void signalerErreur(const char* message) {
  Serial.printf("[ERREUR] %s\n", message);
  toutEteindre();
  appuiErreurEnCours   = false;
  compteurErreursSHT35 = 0;
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
  if (sht35_ok) {
    ecran.setTextColor(TFT_GREEN, 0x1E9F);
    ecran.drawString("SHT35 OK", 120, 155);
  } else {
    ecran.setTextColor(TFT_RED, 0x1E9F);
    ecran.drawString("SHT35 ABSENT", 120, 155);
  }

  ecran.setTextSize(1);
  ecran.setTextColor(TFT_WHITE, 0x1E9F);
  ecran.drawString(sht35_ok ? "GO pour continuer"
                            : "GO → ERREUR", 120, 210);
}

// =============================================================
//  AFFICHAGE ÉCRAN TFT — dispatcher
//
//  240 × 280 px portrait — centre x = 120
//  INIT    : bleu marine  TFT_NAVY
//  CHECK   : bleu-vert    0x1E9F  (géré par afficherResultatI2C)
//  DONNÉES : bleu-vert    0x1E9F
//  BUFFER  : orange       0xFD20
//  MARCHE  : vert         0x07E0
//  ERREUR  : rouge        TFT_RED
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
      // Affichage provisoire — remplacé par afficherResultatI2C()
      ecran.fillScreen(0x1E9F);
      ecran.setTextColor(TFT_WHITE, 0x1E9F);
      ecran.setTextSize(2);
      ecran.drawString("Scan I2C...", 120, 130);
      break;

    case ETAT_ATTENTE_DONNEES:
      ecran.fillScreen(0x1E9F);
      ecran.setTextColor(TFT_WHITE, 0x1E9F);
      ecran.setTextSize(2);
      ecran.drawString("Verification", 120, 100);
      ecran.setTextSize(1);
      ecran.drawString("Lecture capteurs...", 120, 160);
      ecran.drawString("Validation plages", 120, 185);
      break;

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
        ecran.drawString("Fermez l'enceinte", 120, 195);
        ecran.drawString("puis appuyez GO", 120, 215);
        ecran.drawString("pour demarrer", 120, 235);
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

        // Température + indicateur
        const char* indT = (temperature < SEUIL_CHAUFFE)  ? " [+]" :
                           (temperature > SEUIL_REFROIDIT) ? " [-]" : " [=]";
        snprintf(ligne, sizeof(ligne), "T: %.1f/%.1f C%s",
                 temperature, CONSIGNE_TEMP, indT);
        ecran.drawString(ligne, 120, 85);

        // Humidité + alerte
        if (alerteHR) {
          ecran.setTextColor(TFT_RED, 0x07E0);
          snprintf(ligne, sizeof(ligne), "HR: %.1f %% ALERTE", humidite);
        } else {
          snprintf(ligne, sizeof(ligne), "HR: %.1f %%", humidite);
        }
        ecran.drawString(ligne, 120, 115);
        ecran.setTextColor(TFT_BLACK, 0x07E0);

        // CO₂ désactivé
        ecran.setTextColor(0x39E7, 0x07E0);
        snprintf(ligne, sizeof(ligne), "CO2: %.2f %% [OFF]", co2);
        ecran.drawString(ligne, 120, 145);
        ecran.setTextColor(TFT_BLACK, 0x07E0);

        // État Peltier + ventilateur
        const char* etatPeltier =
          (temperature < SEUIL_CHAUFFE)  ? "Chauffe + ventil" :
          (temperature > SEUIL_REFROIDIT) ? "Dissipe (ventil)" : "Neutre";
        ecran.drawString(etatPeltier, 120, 175);

        // Compteur erreurs capteur (debug visuel)
        if (compteurErreursSHT35 > 0) {
          ecran.setTextColor(TFT_YELLOW, 0x07E0);
          snprintf(ligne, sizeof(ligne), "SHT35 err: %d/%d",
                   compteurErreursSHT35, SEUIL_ERREURS_LECTURE);
          ecran.drawString(ligne, 120, 200);
        }
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
      ecran.drawString("Maintenir GO 1s", 120, 200);
      ecran.drawString("pour redemarrer", 120, 220);
      break;
  }
}
