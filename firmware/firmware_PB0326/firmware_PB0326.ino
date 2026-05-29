// =============================================================
//  PB-0326 — Incubateur transportable pour cellules mammifères
//  HEPIA HES-SO — Groupe 3 : Lyla Founas, Mattéo Pozzo di Borgo
//  Version : 3.5 — Mai 2026
// =============================================================
//
//  MODIFICATIONS v3.5 :
//    - GPIO 26 (DAC2) → GPIO 19 pour LED rouge
//    - toutEteindre() en boucle ERREUR → flag one-shot erreurInitialisee
//    - Anti-rebond bouton non bloquant (timestamp) dans tous les états
//    - ATTENTE_DONNEES : lecture capteur forcée à l'entrée de l'état
//    - Ajout ETAT_PAUSE (état 5) :
//        MARCHE → GO maintenu 2s → PAUSE (actionneurs OFF, valeurs live)
//        PAUSE  → GO court       → retour MARCHE
//        PAUSE  → GO maintenu 3s → retour BUFFER (restart propre)
//    - ERREUR devient état 6 (décalage suite à PAUSE)
//    - Validations manuelles GO ajoutées sur ATTENTE_DONNEES → BUFFER
//
//  MACHINE À ÉTATS v3.5 :
//    0 - INIT            : splash 3s automatique
//    1 - CHECK_CAPTEURS  : scan I2C + affichage + GO pour valider
//    2 - ATTENTE_DONNEES : lecture forcée + affichage + GO pour valider
//    3 - BUFFER          : valeurs live + GO pour démarrer
//    4 - MARCHE          : régulation active
//                          GO maintenu 2s → PAUSE
//    5 - PAUSE           : actionneurs OFF, valeurs live
//                          GO court  → retour MARCHE
//                          GO long 3s → retour BUFFER
//    6 - ERREUR          : tout éteint, LED clignote
//                          GO long 1s → ESP.restart()
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
#define BROCHE_PELTIER      32   // MOSFET Grove Peltier
#define BROCHE_VENTILATEUR  14   // Ventilateur
#define BROCHE_SOLENOIDE    27   // Électrovanne V114 — CO₂ DÉSACTIVÉ
#define BROCHE_LED_ROUGE    19   // LED alerte (19 : loin DAC, loin SPI)
#define BROCHE_BOUTON_GO     5   // Bouton GO (INPUT_PULLUP)

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
#define INTERVALLE_ECRAN          2000  // Refresh TFT (ms)
#define INTERVALLE_LECTURE        2000  // Cadence lecture capteurs (ms)
#define DUREE_INIT                3000  // Durée INIT (ms)
#define DUREE_ANTIREBOND            50  // Anti-rebond bouton (ms)
#define DUREE_APPUI_PAUSE         2000  // GO maintenu en MARCHE → PAUSE (ms)
#define DUREE_APPUI_BUFFER_PAUSE  3000  // GO maintenu en PAUSE → BUFFER (ms)
#define DUREE_APPUI_RESET         1000  // GO maintenu en ERREUR → restart (ms)
#define TENTATIVES_I2C               3
#define DELAI_RETRY_I2C             50
#define SEUIL_ERREURS_LECTURE        3  // NaN consécutifs → ERREUR

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
#define THERM_NEUTRE   0
#define THERM_CHAUFFE  1
#define THERM_DISSIPE  2

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
float co2         =  5.0f;  // Valeur fixe SCD40 désactivé
                             // ⚠️ Remettre 0.0f quand SCD40 réactivé

// --- Timers ---
unsigned long derniereLecture = 0;
unsigned long dernierEcran    = 0;
unsigned long tempsInit       = 0;

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
bool scanEffectue = false;
bool scanSHT35OK  = false;
bool scanSCD40OK  = false;

// --- Flag lecture forcée ATTENTE_DONNEES ---
// true si la lecture initiale à l'entrée de l'état a déjà été faite
bool lectureInitialeEffectuee = false;

// --- État thermique mémorisé ---
// -1 = inconnu, force le premier digitalWrite
int etatThermiqueActuel = -1;

// --- Erreur : initialisation one-shot ---
// toutEteindre() n'est appelé qu'une fois à l'entrée de ETAT_ERREUR
// évite les milliers de digitalWrite par seconde en boucle
bool erreurInitialisee = false;

// --- Détection perte capteur ---
uint8_t compteurErreursSHT35 = 0;

// =============================================================
//  STRUCTURE BOUTON — anti-rebond non bloquant
//
//  Remplace les delay(50) bloquants.
//  Trois niveaux de détection par timestamp :
//    appuiCourt  : front descendant + durée >= DUREE_ANTIREBOND
//    appuiLong1  : bouton tenu >= duree1 ms (ex. 1s pour reset ERREUR)
//    appuiLong2  : bouton tenu >= duree2 ms (ex. 2s pour PAUSE, 3s pour BUFFER)
//
//  Utilisation :
//    mettreAJourBouton()     — appeler à chaque début de loop()
//    boutonAppuiCourt()      — true si appui court validé (front montant)
//    boutonAppuiLong(duree)  — true si bouton tenu >= duree ms
// =============================================================
struct Bouton {
  bool     etatActuel    = false;   // true = bouton pressé (LOW inversé)
  bool     etatPrecedent = false;
  bool     appuiDetecte  = false;   // Front descendant détecté
  unsigned long tempsAppui = 0;     // Horodatage début appui
  unsigned long tempsFront = 0;     // Horodatage front descendant
  bool     appuiCourtValide = false; // Appui court consommé
} bouton;

// Met à jour l'état du bouton — appeler en début de loop()
void mettreAJourBouton() {
  bool lu = (digitalRead(BROCHE_BOUTON_GO) == LOW);

  // Front descendant (relâché → pressé)
  if (lu && !bouton.etatPrecedent) {
    bouton.tempsAppui      = millis();
    bouton.appuiDetecte    = true;
    bouton.appuiCourtValide = false;
  }

  // Front montant (pressé → relâché)
  if (!lu && bouton.etatPrecedent) {
    // Valider appui court si durée >= anti-rebond
    if (bouton.appuiDetecte &&
        (millis() - bouton.tempsAppui) >= DUREE_ANTIREBOND &&
        (millis() - bouton.tempsAppui) < DUREE_APPUI_PAUSE) {
      bouton.appuiCourtValide = true;
    }
    bouton.appuiDetecte = false;
  }

  bouton.etatActuel    = lu;
  bouton.etatPrecedent = lu;
}

// Consomme et retourne true si un appui court a été détecté
bool boutonAppuiCourt() {
  if (bouton.appuiCourtValide) {
    bouton.appuiCourtValide = false;
    return true;
  }
  return false;
}

// Retourne true si le bouton est tenu depuis >= duree ms
// Non consommable — reste true tant que le bouton est tenu
bool boutonAppuiLong(unsigned long duree) {
  return bouton.etatActuel &&
         bouton.appuiDetecte &&
         (millis() - bouton.tempsAppui) >= duree;
}

// =============================================================
//  SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("=== PB-0326 Firmware v3.5 ===");

  pinMode(BROCHE_PELTIER,     OUTPUT);
  pinMode(BROCHE_VENTILATEUR, OUTPUT);
  pinMode(BROCHE_SOLENOIDE,   OUTPUT);
  pinMode(BROCHE_LED_ROUGE,   OUTPUT);
  pinMode(BROCHE_BOUTON_GO,   INPUT_PULLUP);

  toutEteindre();

  Wire.begin(21, 22);
  delay(100);

  sht35Demarrer();

  // --- SCD40 : DÉSACTIVÉ ---
  // scd40Demarrer();
  // delay(5000);

  // --- Écran TFT ---
  // Broches dans User_Setup.h : ST7789_DRIVER 240x280
  // MOSI=23 SCLK=18 CS=15 DC=2 RST=33
  ecran.init();
  ecran.setRotation(0);
  ecran.fillScreen(TFT_BLACK);
  ecran.setTextDatum(MC_DATUM);

  // Test blanc 500ms
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

  // --- Mise à jour bouton (anti-rebond non bloquant) ---
  mettreAJourBouton();

  // --- Lecture capteurs cadencée toutes les 2s ---
  if (millis() - derniereLecture >= INTERVALLE_LECTURE) {
    lireCapteurs();
    derniereLecture = millis();
  }

  // --- Refresh TFT au changement d'état ---
  if (etat != etatPrecedent) {
    afficherEcran(etat);
    etatPrecedent = etat;
    // Reset flags d'état à la sortie
    if (etat != ETAT_CHECK_CAPTEURS)  scanEffectue = false;
    if (etat != ETAT_ATTENTE_DONNEES) lectureInitialeEffectuee = false;
    if (etat != ETAT_ERREUR)          erreurInitialisee = false;
    Serial.printf("[ÉTAT] → %d\n", etat);
  }

  switch (etat) {

    // ----------------------------------------------------------
    //  ÉTAT 0 — INIT
    //  Splash screen 3s, passage automatique
    // ----------------------------------------------------------
    case ETAT_INIT:
      if (millis() - tempsInit >= DUREE_INIT) {
        etat = ETAT_CHECK_CAPTEURS;
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 1 — CHECK CAPTEURS
    //
    //  Scan I2C automatique à l'entrée (une fois)
    //  Résultat affiché sur TFT
    //  GO court :
    //    SHT35 OK  → ATTENTE_DONNEES
    //    SHT35 KO  → ERREUR
    // ----------------------------------------------------------
    case ETAT_CHECK_CAPTEURS:
      if (!scanEffectue) {
        verifierCapteursI2C();
        scanEffectue = true;
        Serial.println("[CHECK] Appuyez GO pour continuer");
      }

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
    //  Lecture capteur forcée à l'entrée (pas d'attente 2s)
    //  Affichage des valeurs lues sur TFT
    //  GO court → BUFFER si valeurs plausibles
    //           → ERREUR si valeurs invalides
    //
    //  Permet à l'opérateur de vérifier visuellement les
    //  valeurs initiales avant de démarrer.
    // ----------------------------------------------------------
    case ETAT_ATTENTE_DONNEES:
      // Forcer une lecture immédiate à l'entrée de l'état
      // sans attendre le timer de 2s (sinon valeurs initiales 25°C/50%)
      if (!lectureInitialeEffectuee) {
        lireCapteurs();
        derniereLecture          = millis();  // Resync timer
        lectureInitialeEffectuee = true;
        afficherEcran(ETAT_ATTENTE_DONNEES);  // Redessiner avec vraies valeurs
        Serial.printf("[DONNÉES] T=%.1f HR=%.1f — GO pour valider\n",
                      temperature, humidite);
      }

      // Rafraîchissement périodique
      if (millis() - dernierEcran >= INTERVALLE_ECRAN) {
        afficherEcran(ETAT_ATTENTE_DONNEES);
        dernierEcran = millis();
      }

      if (boutonAppuiCourt()) {
        if (temperature > 10.0f && temperature < 45.0f && humidite > 0.0f) {
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
    //  Valeurs live affichées, opérateur ferme l'enceinte
    //  GO court → MARCHE
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
    //  Régulation active température + humidité
    //  GO maintenu 2s → PAUSE
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

      // GO maintenu 2s → PAUSE
      if (boutonAppuiLong(DUREE_APPUI_PAUSE)) {
        Serial.println("[MARCHE] GO long → PAUSE");
        etat = ETAT_PAUSE;
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 5 — PAUSE
    //  Actionneurs OFF, valeurs live affichées
    //  Permet ouverture enceinte sans déclencher ERREUR
    //
    //  GO court      → retour MARCHE (régulation reprend)
    //  GO maintenu 3s → retour BUFFER (restart propre sans reboot)
    // ----------------------------------------------------------
    case ETAT_PAUSE:
      // Actionneurs éteints (Peltier + ventilateur OFF)
      // toutEteindre() appelé une seule fois via etatPrecedent != etat
      // (la transition vers PAUSE appelle afficherEcran puis ce case)
      if (etatThermiqueActuel != THERM_NEUTRE) {
        digitalWrite(BROCHE_PELTIER,     LOW);
        digitalWrite(BROCHE_VENTILATEUR, LOW);
        etatThermiqueActuel = THERM_NEUTRE;
        Serial.println("[PAUSE] Actionneurs OFF");
      }

      // Rafraîchissement valeurs live
      if (millis() - dernierEcran >= INTERVALLE_ECRAN) {
        afficherEcran(ETAT_PAUSE);
        dernierEcran = millis();
      }

      // GO court → retour MARCHE
      if (boutonAppuiCourt()) {
        Serial.println("[PAUSE] GO → retour MARCHE");
        etatThermiqueActuel  = -1;  // Force recalcul thermique
        compteurErreursSHT35 = 0;   // Reset compteur erreurs
        dernierEcran         = millis();
        etat                 = ETAT_MARCHE;
      }

      // GO maintenu 3s → retour BUFFER (restart propre)
      if (boutonAppuiLong(DUREE_APPUI_BUFFER_PAUSE)) {
        Serial.println("[PAUSE] GO long → retour BUFFER");
        etatThermiqueActuel  = -1;
        compteurErreursSHT35 = 0;
        dernierEcran         = millis();
        etat                 = ETAT_BUFFER;
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 6 — ERREUR
    //  Tout éteint (one-shot), LED clignote
    //  GO maintenu 1s → ESP.restart()
    // ----------------------------------------------------------
    case ETAT_ERREUR:
      // Extinction actionneurs une seule fois à l'entrée
      if (!erreurInitialisee) {
        toutEteindre();
        erreurInitialisee = true;
        Serial.println("[ERREUR] Actionneurs éteints");
      }

      // Clignotement LED non bloquant 500ms
      {
        static unsigned long dernierBlink = 0;
        static bool          ledActive    = false;
        if (millis() - dernierBlink >= 500) {
          ledActive    = !ledActive;
          digitalWrite(BROCHE_LED_ROUGE, ledActive ? HIGH : LOW);
          dernierBlink = millis();
        }
      }

      // GO maintenu 1s → restart
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
//  SCD40 — DÉSACTIVÉ (conservé pour réactivation)
//
//  Pour réactiver :
//    1. Décommenter les corps de scd40EnvoyerCommande() et scd40Demarrer()
//    2. Décommenter scd40Demarrer() + delay(5000) dans setup()
//    3. co2Actif = true
//    4. co2 = 0.0f dans les variables globales
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
//    Lecture valide  → compteur = 0
//    3 NaN en MARCHE → signalerErreur (6s de tolérance)
//    En PAUSE : NaN comptés mais pas de déclenchement ERREUR
//               (enceinte peut être ouverte intentionnellement)
// =============================================================
void lireCapteurs() {
  if (sht35Operationnel) {
    float t = sht35.readTemperature();
    float h = sht35.readHumidity();

    if (!isnan(t) && !isnan(h)) {
      temperature          = t;
      humidite             = h;
      compteurErreursSHT35 = 0;
      Serial.printf("[SHT35] T=%.1fC  HR=%.1f%%\n", temperature, humidite);
    } else {
      compteurErreursSHT35++;
      Serial.printf("[SHT35] NaN (%d/%d)\n",
                    compteurErreursSHT35, SEUIL_ERREURS_LECTURE);
      // Erreur uniquement en MARCHE — pas en PAUSE (ouverture intentionnelle)
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
//  RÉGULATION THERMIQUE — état mémorisé
//
//  digitalWrite émis UNIQUEMENT au changement d'état thermique.
//  Élimine les transitoires répétitifs qui perturbaient le bus I2C.
//
//  T < 36.5°C  → THERM_CHAUFFE : Peltier ON  + Ventilateur ON
//  T > 37.5°C  → THERM_DISSIPE : Peltier OFF + Ventilateur ON
//  Zone neutre → THERM_NEUTRE  : Peltier OFF + Ventilateur OFF
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
  etatThermiqueActuel = -1;
}

// =============================================================
//  SIGNALER ERREUR
// =============================================================
void signalerErreur(const char* message) {
  Serial.printf("[ERREUR] %s\n", message);
  toutEteindre();
  compteurErreursSHT35 = 0;
  erreurInitialisee    = false;  // Sera mis à true dans ETAT_ERREUR
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
  ecran.drawString(sht35_ok ? "GO pour continuer" : "GO → ERREUR", 120, 210);
}

// =============================================================
//  AFFICHAGE ÉCRAN TFT — dispatcher
//  240 × 280 px portrait — centre x = 120
//
//  Couleurs :
//    INIT            : bleu marine  TFT_NAVY
//    CHECK_CAPTEURS  : bleu-vert    0x1E9F
//    ATTENTE_DONNEES : bleu-vert    0x1E9F
//    BUFFER          : orange       0xFD20
//    MARCHE          : vert         0x07E0
//    PAUSE           : orange foncé 0xFC60
//    ERREUR          : rouge        TFT_RED
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
      ecran.drawString("Initialisation...", 120, 185);
      break;

    // --- CHECK CAPTEURS (provisoire pendant scan) ---
    case ETAT_CHECK_CAPTEURS:
      ecran.fillScreen(0x1E9F);
      ecran.setTextColor(TFT_WHITE, 0x1E9F);
      ecran.setTextSize(2);
      ecran.drawString("Scan I2C...", 120, 130);
      break;

    // --- ATTENTE DONNÉES ---
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

        // Indicateur validité
        bool valide = (temperature > 10.0f && temperature < 45.0f && humidite > 0.0f);
        ecran.setTextColor(valide ? TFT_GREEN : TFT_RED, 0x1E9F);
        ecran.drawString(valide ? "Valeurs OK" : "Valeurs INVALIDES", 120, 165);

        ecran.setTextSize(1);
        ecran.setTextColor(TFT_WHITE, 0x1E9F);
        ecran.drawString(valide ? "GO pour continuer"
                                : "GO → ERREUR", 120, 215);
        ecran.drawString("(verifier capteur si invalide)", 120, 235);
        break;
      }

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

        ecran.setTextColor(0x7BEF, 0xFD20);
        snprintf(buf, sizeof(buf), "CO2: %.2f %% [OFF]", co2);
        ecran.drawString(buf, 120, 145);

        ecran.setTextColor(TFT_BLACK, 0xFD20);
        ecran.drawString("Fermez l'enceinte", 120, 190);
        ecran.drawString("GO pour demarrer", 120, 215);
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

        // Compteur erreurs SHT35
        if (compteurErreursSHT35 > 0) {
          ecran.setTextColor(TFT_YELLOW, 0x07E0);
          snprintf(ligne, sizeof(ligne), "SHT35 err: %d/%d",
                   compteurErreursSHT35, SEUIL_ERREURS_LECTURE);
          ecran.drawString(ligne, 120, 192);
        }

        ecran.setTextColor(TFT_BLACK, 0x07E0);
        ecran.drawString("GO 2s pour pause", 120, 255);
        break;
      }

    // --- PAUSE ---
    case ETAT_PAUSE:
      {
        // Orange foncé pour distinguer du BUFFER (orange clair)
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
        ecran.drawString("GO court → reprendre", 120, 215);
        ecran.drawString("GO 3s   → retour PRET", 120, 235);
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
      ecran.drawString("GO 1s → redemarrer", 120, 200);
      break;
  }
}
