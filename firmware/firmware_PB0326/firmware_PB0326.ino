// =============================================================
//  PB-0326 — Incubateur transportable pour cellules mammifères
//  HEPIA HES-SO — Groupe 3 : Lyla Founas, Mattéo Pozzo di Borgo
//  Version : 3.2 — Mai 2026
// =============================================================
//
//  DESCRIPTION GÉNÉRALE :
//  Ce firmware contrôle un incubateur portable destiné au transport
//  de cellules mammifères vivantes (~1 heure). Il maintient trois
//  paramètres biologiques critiques :
//    - Température  : 37°C ± 0.5°C  (via Peltier TEC1-12703)
//    - Humidité     : ~95-99% HR     (monitoring passif, éponge saturée)
//    - CO₂          : 5.0% ± 0.3%   (injection via électrovanne SMC V114)
//
//  ARCHITECTURE LOGICIELLE :
//  Machine à états principale à 6 états (0→5) :
//    0 - INIT            : initialisation matérielle
//    1 - CHECK_CAPTEURS  : scan I2C + affichage + attente bouton GO
//    2 - ATTENTE_DONNEES : validation plages capteurs
//    3 - BUFFER          : attente appui bouton GO
//    4 - MARCHE          : régulation active
//    5 - ERREUR          : sécurité, tout éteint
//                          → appui bouton GO (1s) pour redémarrer
//
//  CAPTEURS :
//    - SHT35  (I2C 0x44) : température + humidité → Adafruit SHT31
//    - SCD40  (I2C 0x62) : CO₂ → DÉSACTIVÉ (commenté, réactivable)
//
//  ACTIONNEURS :
//    - Peltier TEC1-12703 : chauffage seul (MOSFET Grove, ON/OFF)
//    - Ventilateur        : ON pendant chauffage ET si T > 37.5°C
//    - Électrovanne V114  : CO₂ → DÉSACTIVÉE (co2Actif = false)
//
//  ÉCRAN :
//    - 1.69" TFT ST7789V3 240x280 RGB
//    - Configuré via defines en tête de fichier
//    - ⚠️ Ces defines doivent correspondre à ton câblage SPI physique
//
//  MODIFICATIONS v3.2 :
//    - Écran ST7789V3 240×280 configuré (defines TFT_eSPI)
//    - Ventilateur ON aussi pendant le chauffage (visuel)
//    - ETAT_ERREUR : appui bouton GO (1s) → ESP.restart()
//    - Correction logique ETAT_ERREUR si SHT35 absent
//
// =============================================================

// =============================================================
//  CONFIGURATION ÉCRAN TFT ST7789V3 — 1.69" 240x280
//
//  Ces defines surchargent User_Setup.h si TFT_eSPI le permet.
//  ⚠️ SI L'ÉCRAN NE FONCTIONNE TOUJOURS PAS :
//     Modifier directement le fichier User_Setup.h dans le dossier
//     de la bibliothèque TFT_eSPI avec les mêmes valeurs ci-dessous.
//
//  Broches SPI ESP32 standard :
//    MOSI = GPIO 23
//    SCLK = GPIO 18
//    CS   = GPIO 15  (à adapter selon ton câblage)
//    DC   = GPIO  2  (à adapter selon ton câblage)
//    RST  = GPIO  4  (à adapter selon ton câblage — attention: même que PELTIER ici, à changer)
//
//  ⚠️ IMPORTANT : vérifie que CS/DC/RST ne sont pas en conflit
//     avec tes autres broches. Adapter si nécessaire.
// =============================================================


#include <TFT_eSPI.h>        // Bibliothèque écran TFT SPI
#include <Wire.h>            // Bus I2C pour SHT35 et SCD40
#include <Adafruit_SHT31.h>  // Bibliothèque capteur température/humidité SHT35

// =============================================================
//  BROCHES GPIO — Attribution finale sans conflit
//  I2C  : SDA=21, SCL=22 (défaut ESP32, réservés)
//  SPI  : MOSI=23, SCLK=18, CS=15, DC=2, RST=33 (écran)
//  EVITER: 6-11 (flash SPI), 34/35/36/39 (input only)
// =============================================================
#define BROCHE_PELTIER      32   // MOSFET Grove Peltier — chauffage ON/OFF
#define BROCHE_VENTILATEUR  13   // Ventilateur dissipateur (ON/OFF)
#define BROCHE_SOLENOIDE    27   // Électrovanne V114 via MOSFET — CO₂ DÉSACTIVÉ
#define BROCHE_LED_ROUGE    26   // LED alerte erreur (clignotante en état 5)
#define BROCHE_BOUTON_GO     5   // Bouton démarrage / reset erreur (INPUT_PULLUP)

// =============================================================
//  CONSIGNES BIOLOGIQUES
// =============================================================
#define CONSIGNE_TEMP   37.0f  // Température cible en °C
#define SEUIL_CHAUFFE   36.5f  // En dessous → Peltier + ventilateur ON
#define SEUIL_REFROIDIT 37.5f  // Au dessus  → Peltier OFF + ventilateur ON
#define TEMP_SECURITE   40.0f  // Sécurité absolue → passage ERREUR

#define CONSIGNE_CO2    5.0f   // CO₂ cible en % (non utilisé, SCD40 désactivé)
#define SEUIL_BAS_CO2   4.7f   // En dessous → injection CO₂
#define SEUIL_HAUT_CO2  5.3f   // Au dessus  → dilution naturelle

#define SEUIL_HR_ALERTE 85.0f  // En dessous → alerte enceinte ouverte

// =============================================================
//  PARAMÈTRES CO₂ — conservés, non utilisés (SCD40 désactivé)
// =============================================================
#define DUREE_INJECT_MS     200    // Durée ouverture électrovanne (ms)
#define DUREE_STABILISATION 30000  // Attente post-injection (ms)
#define INTERVALLE_CO2      5000   // Cadence lecture SCD40 (ms)

// =============================================================
//  PARAMÈTRES DIVERS
// =============================================================
#define INTERVALLE_ECRAN    2000  // Rafraîchissement écran TFT (ms)
#define INTERVALLE_LECTURE  2000  // Cadence lecture capteurs (ms) — anti-spam
#define DUREE_INIT          3000  // Durée état INIT (ms)
#define DUREE_APPUI_RESET   1000  // Durée appui bouton GO pour reset ERREUR (ms)
#define TENTATIVES_I2C         3  // Tentatives scan I2C avant abandon
#define DELAI_RETRY_I2C       50  // Délai entre tentatives I2C (ms)

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
//  ÉTATS INTERNES CO₂ — conservés, inactifs
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
unsigned long derniereLecture = 0;  // Timer lecture capteurs (anti-spam)
unsigned long dernierEcran    = 0;  // Timer refresh TFT
unsigned long tempsInit       = 0;  // Timer INIT

// --- CO₂ ---
int           etatCO2    = CO2_IDLE;
unsigned long timerCO2   = 0;
unsigned long dernierCO2 = 0;

// --- Flags ---
bool sht35Operationnel = false;  // true si SHT35 détecté et initialisé
bool co2Actif          = false;  // ⚠️ Mettre true pour activer CO₂
bool scd40Demarre      = false;  // ⚠️ Mis à true par scd40Demarrer()
bool alerteHR          = false;  // true si HR < SEUIL_HR_ALERTE

// --- Scan I2C ---
bool scanEffectue = false;  // true si scan déjà fait dans CHECK_CAPTEURS
bool scanSHT35OK  = false;  // Résultat scan SHT35
bool scanSCD40OK  = false;  // Résultat scan SCD40 (false = désactivé)

// --- Reset erreur ---
// Détection appui long bouton GO dans ETAT_ERREUR
unsigned long tempsAppuiErreur = 0;   // Horodatage début appui en état ERREUR
bool          appuiErreurEnCours = false; // true si bouton tenu enfoncé

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("=== PB-0326 Firmware v3.2 ===");

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
  // Décommenter pour réactiver :
  // scd40Demarrer();
  // delay(5000);

  // --- Écran TFT ST7789V3 ---
  // ⚠️ Si l'écran reste noir après init :
  //    1. Vérifier le câblage CS/DC/RST/MOSI/SCLK
  //    2. Modifier TFT_CS, TFT_DC, TFT_RST ci-dessus selon ton câblage
  //    3. Essayer TFT_RGB_ORDER TFT_BGR si les couleurs sont inversées
  //    4. En dernier recours : modifier User_Setup.h dans bibliothèque TFT_eSPI
  ecran.init();
  ecran.setRotation(0);         // 0 = portrait 240×280, 1 = paysage 280×240
  ecran.fillScreen(TFT_BLACK);
  ecran.setTextDatum(MC_DATUM); // Texte centré par défaut

  // --- Test écran au démarrage ---
  // Affiche un rectangle blanc 1 seconde pour confirmer que l'écran fonctionne
  ecran.fillScreen(TFT_WHITE);
  delay(500);
  ecran.fillScreen(TFT_BLACK);

  // --- Timer INIT ---
  tempsInit = millis();

  Serial.println("[SETUP] Initialisation terminée");
}

// =============================================================
//  LOOP — Machine à états principale
// =============================================================
void loop() {

  // --- Lecture capteurs cadencée toutes les 2s ---
  if (millis() - derniereLecture >= INTERVALLE_LECTURE) {
    lireCapteurs();
    derniereLecture = millis();
  }

  // --- Refresh TFT au changement d'état ---
  if (etat != etatPrecedent) {
    afficherEcran(etat);
    etatPrecedent = etat;
    if (etat != ETAT_CHECK_CAPTEURS) {
      scanEffectue = false;
    }
    Serial.printf("[ÉTAT] Transition → %d\n", etat);
  }

  // --- Dispatch états ---
  switch (etat) {

    // ----------------------------------------------------------
    //  ÉTAT 0 — INIT
    //  Durée fixe 3s puis passage automatique
    // ----------------------------------------------------------
    case ETAT_INIT:
      if (millis() - tempsInit >= DUREE_INIT) {
        Serial.println("[INIT] → CHECK_CAPTEURS");
        etat = ETAT_CHECK_CAPTEURS;
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 1 — CHECK CAPTEURS
    //
    //  Phase 1 (automatique) : scan I2C SHT35 + affichage résultat
    //  Phase 2 (manuel)      : attente bouton GO
    //    - SHT35 OK  → ATTENTE_DONNEES
    //    - SHT35 KO  → ERREUR (capteur température obligatoire)
    //  SCD40 absent = non bloquant (co2Actif reste false)
    // ----------------------------------------------------------
    case ETAT_CHECK_CAPTEURS:
      if (!scanEffectue) {
        verifierCapteursI2C();
        scanEffectue = true;
        Serial.println("[CHECK] Scan terminé — appuyez GO pour continuer");
      }

      if (digitalRead(BROCHE_BOUTON_GO) == LOW) {
        delay(50);
        if (digitalRead(BROCHE_BOUTON_GO) == LOW) {
          if (scanSHT35OK) {
            Serial.println("[CHECK] SHT35 OK → ATTENTE_DONNEES");
            etat = ETAT_ATTENTE_DONNEES;
          } else {
            Serial.println("[CHECK] SHT35 ABSENT → ERREUR");
            signalerErreur("SHT35 requis");
          }
        }
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 2 — ATTENTE DONNÉES
    //  Valide que SHT35 donne des valeurs plausibles
    //  Passage automatique sans bouton
    // ----------------------------------------------------------
    case ETAT_ATTENTE_DONNEES:
      if (temperature > 10.0f && temperature < 45.0f && humidite > 0.0f) {
        Serial.println("[DONNÉES] OK → BUFFER");
        etat = ETAT_BUFFER;
      } else {
        Serial.printf("[DONNÉES] Invalides T=%.1f HR=%.1f → ERREUR\n",
                      temperature, humidite);
        signalerErreur("Donnees invalides");
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 3 — BUFFER
    //  Affichage valeurs live + attente GO
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
          etat         = ETAT_MARCHE;
        }
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 4 — MARCHE
    //  Régulation thermique + humidité (CO₂ désactivé)
    // ----------------------------------------------------------
    case ETAT_MARCHE:
      if (temperature >= TEMP_SECURITE) {
        Serial.printf("[SÉCURITÉ] T=%.1f°C → ERREUR\n", temperature);
        signalerErreur("TEMP CRITIQUE");
        break;
      }

      regulerTemperature();
      miseAJourCO2();       // Retourne immédiatement si co2Actif = false
      miseAJourHumidite();

      if (millis() - dernierEcran >= INTERVALLE_ECRAN) {
        afficherEcran(ETAT_MARCHE);
        dernierEcran = millis();
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 5 — ERREUR
    //  Tout éteint, LED rouge clignotante
    //
    //  Sortie de l'état ERREUR :
    //    Appui bouton GO maintenu >= DUREE_APPUI_RESET (1s)
    //    → ESP.restart() — redémarrage complet du firmware
    //
    //  Un appui court est ignoré (évite reset accidentel)
    // ----------------------------------------------------------
    case ETAT_ERREUR:
      toutEteindre();

      // Clignotement LED rouge non bloquant (500ms)
      {
        static unsigned long dernierBlink = 0;
        static bool          ledActive    = false;
        if (millis() - dernierBlink >= 500) {
          ledActive    = !ledActive;
          digitalWrite(BROCHE_LED_ROUGE, ledActive ? HIGH : LOW);
          dernierBlink = millis();
        }
      }

      // Détection appui long bouton GO pour reset
      if (digitalRead(BROCHE_BOUTON_GO) == LOW) {
        if (!appuiErreurEnCours) {
          // Début de l'appui — mémoriser le temps
          appuiErreurEnCours = true;
          tempsAppuiErreur   = millis();
          Serial.println("[ERREUR] Bouton maintenu — relâcher après 1s pour reset");
        } else {
          // Bouton toujours tenu — vérifier durée
          if (millis() - tempsAppuiErreur >= DUREE_APPUI_RESET) {
            Serial.println("[ERREUR] Reset demandé → redémarrage");
            delay(200);
            ESP.restart();  // Redémarrage complet de l'ESP32
          }
        }
      } else {
        // Bouton relâché — annuler le comptage
        if (appuiErreurEnCours) {
          Serial.println("[ERREUR] Appui trop court — maintenir 1s pour reset");
        }
        appuiErreurEnCours = false;
      }
      break;
  }
}

// =============================================================
//  SHT35 — Démarrage via bibliothèque Adafruit
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
//  SCD40 — Envoi commande I2C brute — DÉSACTIVÉ
// =============================================================
void scd40EnvoyerCommande(uint16_t cmd) {
  // ⚠️ DÉSACTIVÉ
  // Wire.beginTransmission(ADRESSE_SCD40);
  // Wire.write((uint8_t)(cmd >> 8));
  // Wire.write((uint8_t)(cmd & 0xFF));
  // Wire.endTransmission();
}

// =============================================================
//  SCD40 — Démarrage mesure périodique — DÉSACTIVÉ
//
//  Pour réactiver :
//    1. Décommenter le corps
//    2. Décommenter scd40Demarrer() dans setup()
//    3. co2Actif = true
//    4. co2 = 0.0f dans variables globales
//    5. Décommenter bloc SCD40 dans verifierCapteursI2C()
// =============================================================
void scd40Demarrer() {
  // ⚠️ DÉSACTIVÉ
  // scd40EnvoyerCommande(SCD40_CMD_START_MEAS);
  // scd40Demarre = true;
  // Serial.println("[SCD40] Démarré");
}

// =============================================================
//  SCD40 — CRC-8 — CONSERVÉ pour réactivation
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
//  SCD40 — Lecture mesure — DÉSACTIVÉE
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
  // Serial.printf("[SCD40] CO2=%.0fppm T=%.1fC HR=%.1f%%\n", co2_out, temp_out, hum_out);
  // return true;
  return false;
}

// =============================================================
//  LECTURE CAPTEURS — cadencée toutes les 2s
//
//  Appelée depuis loop() via timer derniereLecture.
//  Évite la saturation du bus I2C et du Serial.
//
//  Actif   : SHT35 → temperature + humidite
//  Inactif : SCD40 → co2 reste à 5.0f (valeur fixe)
// =============================================================
void lireCapteurs() {
  if (sht35Operationnel) {
    float t = sht35.readTemperature();
    float h = sht35.readHumidity();
    if (!isnan(t)) temperature = t;
    if (!isnan(h)) humidite    = h;
    Serial.printf("[SHT35] T=%.1fC  HR=%.1f%%\n", temperature, humidite);
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
    Serial.printf("[I2C] Tentative %d/%d 0x%02X erreur=%d\n",
                  i + 1, tentatives, adresse, erreur);
    if (i < tentatives - 1) delay(DELAI_RETRY_I2C);
  }
  return false;
}

// =============================================================
//  VÉRIFICATION CAPTEURS I2C
//
//  Scan SHT35 obligatoire + SCD40 optionnel (désactivé).
//  Met à jour scanSHT35OK, scanSCD40OK, sht35Operationnel.
//  Affiche le résultat sur TFT.
//  La décision de continuer est prise par le bouton GO
//  dans ETAT_CHECK_CAPTEURS.
// =============================================================
void verifierCapteursI2C() {
  Serial.println("[I2C] Scan...");

  // SHT35 obligatoire
  scanSHT35OK       = scannerAdresse(ADRESSE_SHT35, TENTATIVES_I2C);
  sht35Operationnel = scanSHT35OK;
  Serial.printf("[I2C] SHT35 → %s\n", scanSHT35OK ? "OK" : "ABSENT");

  // SCD40 désactivé — décommenter pour réactiver
  // ⚠️ DÉSACTIVÉ
  // scanSCD40OK = scannerAdresse(ADRESSE_SCD40, TENTATIVES_I2C);
  // if (scanSCD40OK) co2Actif = true;
  // Serial.printf("[I2C] SCD40 → %s\n", scanSCD40OK ? "OK" : "ABSENT");
  scanSCD40OK = false;

  afficherResultatI2C(scanSHT35OK, scanSCD40OK);
}

// =============================================================
//  RÉGULATION THERMIQUE — Bang-bang avec zone morte
//
//  T < 36.5°C  → Peltier ON  + Ventilateur ON  (chauffe + visuel)
//  T > 37.5°C  → Peltier OFF + Ventilateur ON  (dissipation passive)
//  Zone neutre → Peltier OFF + Ventilateur OFF
//
//  Ventilateur actif pendant la chauffe :
//    Permet la circulation d'air chaud dans l'enceinte
//    et fournit un retour visuel que le chauffage est actif.
// =============================================================
void regulerTemperature() {
  if (temperature < SEUIL_CHAUFFE) {
    // Chauffe — Peltier ON + ventilateur ON (circulation + visuel)
    digitalWrite(BROCHE_PELTIER,     HIGH);
    digitalWrite(BROCHE_VENTILATEUR, HIGH);
    Serial.printf("[THERM] Chauffe — T=%.1fC\n", temperature);

  } else if (temperature > SEUIL_REFROIDIT) {
    // Trop chaud — Peltier OFF + ventilateur ON (dissipation)
    digitalWrite(BROCHE_PELTIER,     LOW);
    digitalWrite(BROCHE_VENTILATEUR, HIGH);
    Serial.printf("[THERM] Trop chaud — T=%.1fC\n", temperature);

  } else {
    // Zone neutre — tout OFF
    digitalWrite(BROCHE_PELTIER,     LOW);
    digitalWrite(BROCHE_VENTILATEUR, LOW);
  }
}

// =============================================================
//  RÉGULATION CO₂ — DÉSACTIVÉE (co2Actif = false)
//
//  Retourne immédiatement tant que co2Actif = false.
//  Vanne reste LOW (V114 NC = fermée = sécurité).
//  Sous-machine conservée intacte pour réactivation.
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
//  MONITORING HUMIDITÉ — passif avec alerte
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
//  TOUT ÉTEINDRE — sécurité complète
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
  appuiErreurEnCours = false;  // Reset flag appui
  etat = ETAT_ERREUR;
  afficherEcran(ETAT_ERREUR);
}

// =============================================================
//  AFFICHAGE RÉSULTAT I2C
//
//  Écran dédié à ETAT_CHECK_CAPTEURS après le scan.
//  SHT35 : vert si OK, rouge si absent
//  SCD40 : gris "DESACTIVE" (toujours, tant que désactivé)
//  Instruction : "Maintenez GO pour continuer"
// =============================================================
void afficherResultatI2C(bool sht35_ok, bool scd40_ok) {
  ecran.fillScreen(0x1E9F);
  ecran.setTextDatum(MC_DATUM);

  ecran.setTextColor(TFT_WHITE, 0x1E9F);
  ecran.setTextSize(2);
  ecran.drawString("Capteurs I2C", 120, 25);

  ecran.setTextSize(1);

  // SHT35
  ecran.setTextColor(sht35_ok ? TFT_GREEN : TFT_RED, 0x1E9F);
  ecran.drawString(sht35_ok ? "SHT35 [0x44]  OK"
                            : "SHT35 [0x44]  ABSENT", 120, 80);

  // SCD40 — gris "DESACTIVE"
  ecran.setTextColor(0xC618, 0x1E9F);  // Gris
  ecran.drawString("SCD40 [0x62]  DESACTIVE", 120, 105);

  // Résultat global
  ecran.setTextSize(2);
  if (sht35_ok) {
    ecran.setTextColor(TFT_GREEN, 0x1E9F);
    ecran.drawString("SHT35 OK", 120, 150);
  } else {
    ecran.setTextColor(TFT_RED, 0x1E9F);
    ecran.drawString("SHT35 ABSENT", 120, 150);
  }

  // Instruction bouton
  ecran.setTextSize(1);
  ecran.setTextColor(TFT_WHITE, 0x1E9F);
  ecran.drawString(sht35_ok ? "GO pour continuer"
                            : "GO → passage ERREUR", 120, 200);
  ecran.drawString("(appui court suffit)", 120, 220);
}

// =============================================================
//  AFFICHAGE ÉCRAN TFT — dispatcher principal
//
//  Dimensions : 240 × 280 px (portrait ST7789V3)
//  Centre horizontal : x = 120
//
//  Couleurs :
//    INIT            : bleu marine (TFT_NAVY)
//    CHECK_CAPTEURS  : bleu-vert   (0x1E9F) — géré par afficherResultatI2C
//    ATTENTE_DONNÉES : bleu-vert   (0x1E9F)
//    BUFFER          : orange      (0xFD20)
//    MARCHE          : vert        (0x07E0)
//    ERREUR          : rouge       (TFT_RED)
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
      ecran.drawString("Initialisation...", 120, 180);
      break;

    // --- CHECK CAPTEURS ---
    // Écran géré par afficherResultatI2C() après le scan
    // Cet affichage est provisoire pendant le scan (fraction de seconde)
    case ETAT_CHECK_CAPTEURS:
      ecran.fillScreen(0x1E9F);
      ecran.setTextColor(TFT_WHITE, 0x1E9F);
      ecran.setTextSize(2);
      ecran.drawString("Scan I2C...", 120, 130);
      break;

    // --- ATTENTE DONNÉES ---
    case ETAT_ATTENTE_DONNEES:
      ecran.fillScreen(0x1E9F);
      ecran.setTextColor(TFT_WHITE, 0x1E9F);
      ecran.setTextSize(2);
      ecran.drawString("Verification", 120, 100);
      ecran.setTextSize(1);
      ecran.drawString("Lecture capteurs...", 120, 160);
      ecran.drawString("Validation plages", 120, 185);
      break;

    // --- BUFFER ---
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

        // CO₂ — valeur fixe, indiqué désactivé
        ecran.setTextColor(0x7BEF, 0xFD20);
        snprintf(buf, sizeof(buf), "CO2: %.2f %% [OFF]", co2);
        ecran.drawString(buf, 120, 145);

        ecran.setTextColor(TFT_BLACK, 0xFD20);
        ecran.drawString("Fermez l'enceinte", 120, 195);
        ecran.drawString("puis appuyez GO", 120, 215);
        ecran.drawString("pour demarrer", 120, 235);
        break;
      }

    // --- MARCHE ---
    case ETAT_MARCHE:
      {
        ecran.fillScreen(0x07E0);
        ecran.setTextColor(TFT_BLACK, 0x07E0);
        ecran.setTextSize(2);
        ecran.drawString("EN MARCHE", 120, 25);

        ecran.setTextSize(1);
        char ligne[32];

        // Température + indicateur état Peltier
        const char* indT = (temperature < SEUIL_CHAUFFE)   ? " [+]" :
                           (temperature > SEUIL_REFROIDIT)  ? " [-]" : " [=]";
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
        bool chauffe  = (temperature < SEUIL_CHAUFFE);
        bool dissipe  = (temperature > SEUIL_REFROIDIT);
        const char* etatPeltier = chauffe  ? "Chauffe + ventil" :
                                  dissipe  ? "Dissipe (ventil)" : "Neutre";
        snprintf(ligne, sizeof(ligne), "%s", etatPeltier);
        ecran.drawString(ligne, 120, 175);
        break;
      }

    // --- ERREUR ---
    case ETAT_ERREUR:
      ecran.fillScreen(TFT_RED);
      ecran.setTextColor(TFT_WHITE, TFT_RED);
      ecran.setTextSize(2);
      ecran.drawString("ERREUR", 120, 70);
      ecran.setTextSize(1);
      ecran.drawString("Systeme arrete", 120, 130);
      ecran.drawString("Actionneurs OFF", 120, 155);
      ecran.drawString("Maintenir GO 1s", 120, 195);
      ecran.drawString("pour redemarrer", 120, 215);
      break;
  }
}
