// =============================================================
//  PB-0326 — Incubateur transportable pour cellules mammifères
//  HEPIA HES-SO — Groupe 3 : Lyla Founas, Mattéo Pozzo di Borgo
//  Version : 3.1 — Mai 2026
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
//    1 - CHECK_CAPTEURS  : vérification présence I2C (3 tentatives)
//                          → validation par bouton GO avant de continuer
//    2 - ATTENTE_DONNEES : validation plages capteurs
//    3 - BUFFER          : attente appui bouton GO
//    4 - MARCHE          : régulation active
//    5 - ERREUR          : sécurité, tout éteint
//
//  CAPTEURS RETENUS :
//    - SHT35  (I2C 0x44) : température + humidité → bibliothèque Adafruit
//    - SCD40  (I2C 0x62) : CO₂ → DÉSACTIVÉ (commenté, à réactiver)
//
//  ACTIONNEURS :
//    - Peltier TEC1-12703 : chauffage seul (MOSFET Grove, ON/OFF)
//    - Ventilateur        : ON si T > 37.5°C (dissipation passive)
//    - Électrovanne V114  : CO₂ → DÉSACTIVÉE (co2Actif = false)
//
//  MODIFICATION v3.1 :
//    - Partie CO₂/SCD40 mise en commentaire (réactivable)
//    - lireCapteurs() limité à 1 appel toutes les INTERVALLE_LECTURE ms
//    - CHECK_CAPTEURS : affichage résultat + attente bouton GO
//      avant de passer à l'état suivant
//    - Broches corrigées (évite GPIO input-only et flash SPI)
//
// =============================================================

#include <TFT_eSPI.h>        // Bibliothèque écran TFT SPI
#include <Wire.h>            // Bus I2C pour SHT35 et SCD40
#include <Adafruit_SHT31.h>  // Bibliothèque capteur température/humidité SHT35

// =============================================================
//  BROCHES GPIO — Attribution finale sans conflit
//  I2C  : SDA=21, SCL=22 (défaut ESP32, réservés)
//  EVITER: 6-11 (flash SPI), 34/35/36/39 (input only)
// =============================================================
#define BROCHE_PELTIER      32   // MOSFET Grove Peltier — chauffage ON/OFF
#define BROCHE_VENTILATEUR  25   // Ventilateur dissipateur (ON/OFF)
#define BROCHE_SOLENOIDE    27   // Électrovanne V114 via MOSFET — CO₂ DÉSACTIVÉ
#define BROCHE_LED_ROUGE    26   // LED alerte erreur (clignotante en état 5)
#define BROCHE_BOUTON_GO    5   // Bouton démarrage transport (INPUT_PULLUP)

// =============================================================
//  CONSIGNES BIOLOGIQUES
//  Valeurs cibles pour maintien de la viabilité cellulaire
// =============================================================
#define CONSIGNE_TEMP   37.0f  // Température cible en °C
#define SEUIL_CHAUFFE   36.5f  // En dessous → Peltier chauffe
#define SEUIL_REFROIDIT 37.5f  // Au dessus  → Peltier OFF + ventilateur
#define TEMP_SECURITE   40.0f  // Sécurité absolue → passage ERREUR

// --- CO₂ : paramètres conservés mais non utilisés (SCD40 désactivé) ---
#define CONSIGNE_CO2    5.0f   // CO₂ cible en %
#define SEUIL_BAS_CO2   4.7f   // En dessous → injection CO₂
#define SEUIL_HAUT_CO2  5.3f   // Au dessus  → attente dilution naturelle

#define SEUIL_HR_ALERTE 85.0f  // En dessous → alerte enceinte ouverte

// =============================================================
//  PARAMÈTRES RÉGULATION CO₂
//  Conservés en commentaire — réactiver avec SCD40
// =============================================================
#define DUREE_INJECT_MS     200    // Durée ouverture électrovanne (ms) — à calibrer
#define DUREE_STABILISATION 30000  // Attente post-injection (ms)
#define INTERVALLE_CO2      5000   // Cadence lecture SCD40 (ms)

// =============================================================
//  PARAMÈTRES AFFICHAGE & DIVERS
// =============================================================
#define INTERVALLE_ECRAN   2000  // Rafraîchissement écran TFT (ms)
#define INTERVALLE_LECTURE 2000  // Cadence lecture capteurs (ms) — anti-spam
#define DUREE_INIT         3000  // Durée état INIT (ms)
#define TENTATIVES_I2C        3  // Nombre de tentatives scan I2C avant erreur
#define DELAI_RETRY_I2C      50  // Délai entre tentatives I2C (ms)

// =============================================================
//  ÉTATS MACHINE PRINCIPALE
// =============================================================
#define ETAT_INIT            0  // Initialisation matérielle
#define ETAT_CHECK_CAPTEURS  1  // Vérification présence capteurs I2C
#define ETAT_ATTENTE_DONNEES 2  // Validation plages de valeurs capteurs
#define ETAT_BUFFER          3  // Attente appui bouton GO par l'opérateur
#define ETAT_MARCHE          4  // Régulation active — transport en cours
#define ETAT_ERREUR          5  // Erreur critique — tout éteint

// =============================================================
//  ÉTATS INTERNES RÉGULATION CO₂
//  Conservés — sous-machine inactive tant que co2Actif = false
// =============================================================
#define CO2_IDLE        0  // En attente, lecture périodique active
#define CO2_INJECTING   1  // Électrovanne ouverte, injection en cours
#define CO2_STABILIZING 2  // Post-injection, attente diffusion CO₂

// =============================================================
//  ADRESSES I2C CAPTEURS
// =============================================================
#define ADRESSE_SHT35 0x44  // Adresse I2C SHT35 (défaut)
#define ADRESSE_SCD40 0x62  // Adresse I2C SCD40 (défaut) — DÉSACTIVÉ

// =============================================================
//  COMMANDES SCD40 — DÉSACTIVÉES (SCD40 non branché)
//  Décommenter quand SCD40 disponible
// =============================================================
#define SCD40_CMD_START_MEAS 0x21B1  // Démarrer mesure périodique
#define SCD40_CMD_READ_MEAS  0xEC05  // Lire mesure courante
#define SCD40_CMD_STOP_MEAS  0x3F86  // Arrêter mesure périodique

// =============================================================
//  OBJETS
// =============================================================
TFT_eSPI       ecran = TFT_eSPI();       // Écran TFT (SPI, bibliothèque TFT_eSPI)
Adafruit_SHT31 sht35 = Adafruit_SHT31(); // Capteur SHT35 (I2C, bibliothèque Adafruit)

// =============================================================
//  VARIABLES GLOBALES — ÉTAT SYSTÈME
// =============================================================

// --- Machine à états principale ---
int etat          = ETAT_INIT;  // État courant de la machine principale
int etatPrecedent = -1;         // Mémorise l'état précédent pour détecter les transitions

// --- Valeurs capteurs (mises à jour par lireCapteurs()) ---
float temperature = 25.0f;  // Température mesurée par SHT35 (°C)
float humidite    = 50.0f;  // Humidité relative mesurée par SHT35 (%)
float co2         =  5.0f;  // CO₂ — valeur fixe 5.0% car SCD40 désactivé
                             // ⚠️ Remettre à 0.0f quand SCD40 réactivé

// --- Timer lecture capteurs (anti-spam) ---
// lireCapteurs() n'est appelé que toutes les INTERVALLE_LECTURE ms
// Évite la saturation du bus I2C et du port série
unsigned long derniereLecture = 0;

// --- État régulation CO₂ ---
// Sous-machine conservée mais inactive (co2Actif = false)
int           etatCO2    = CO2_IDLE;  // Sous-état de la régulation CO₂
unsigned long timerCO2   = 0;         // Horodatage dernier événement CO₂ (ms)
unsigned long dernierCO2 = 0;         // Horodatage dernière lecture CO₂ (ms)

// --- Timers non bloquants ---
unsigned long dernierEcran = 0;  // Horodatage dernier rafraîchissement TFT (ms)
unsigned long tempsInit    = 0;  // Horodatage début état INIT (ms)

// --- Flags capteurs ---
bool sht35Operationnel = false;  // true si SHT35 détecté et initialisé

// --- Flag CO₂ : DÉSACTIVÉ — passer à true quand SCD40 branché ---
// Tant que false : miseAJourCO2() retourne immédiatement sans rien faire
// La vanne reste fermée (sécurité par défaut V114 NC)
bool co2Actif = false;  // ⚠️ Mettre à true pour activer la régulation CO₂

// --- Flag SCD40 démarré ---
// DÉSACTIVÉ — laisser false tant que SCD40 non branché
bool scd40Demarre = false;  // ⚠️ Mis à true par scd40Demarrer() quand réactivé

// --- Flag humidité ---
bool alerteHR = false;  // true si HR sous seuil d'alerte (enceinte ouverte ?)

// --- Flag résultat scan I2C (utilisé dans CHECK_CAPTEURS) ---
// Mémorise le résultat du scan pour affichage et validation bouton
bool scanEffectue    = false;  // true si le scan I2C a déjà été fait dans cet état
bool scanSHT35OK     = false;  // Résultat scan SHT35
bool scanSCD40OK     = false;  // Résultat scan SCD40 (toujours false si désactivé)

// =============================================================
//  SETUP — Initialisation unique au démarrage
// =============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("=== PB-0326 Firmware v3.1 ===");

  // --- Configuration broches de sortie ---
  pinMode(BROCHE_PELTIER,     OUTPUT);
  pinMode(BROCHE_VENTILATEUR, OUTPUT);
  pinMode(BROCHE_SOLENOIDE,   OUTPUT);
  pinMode(BROCHE_LED_ROUGE,   OUTPUT);

  // --- Configuration broche d'entrée ---
  pinMode(BROCHE_BOUTON_GO, INPUT_PULLUP);  // Résistance pull-up interne activée

  // --- Sécurité : tout éteindre avant toute chose ---
  toutEteindre();

  // --- Initialisation bus I2C ---
  // SDA=21, SCL=22 — broches I2C par défaut ESP32
  // Doit être appelé AVANT toute utilisation de Wire ou bibliothèque I2C
  Wire.begin(21, 22);
  delay(100);  // Stabilisation bus I2C

  // --- Initialisation SHT35 via bibliothèque Adafruit ---
  sht35Demarrer();

  // --- SCD40 : DÉSACTIVÉ ---
  // Décommenter les deux lignes suivantes pour réactiver le SCD40 :
  // scd40Demarrer();
  // delay(5000);  // Attente première mesure SCD40 disponible

  // --- Initialisation écran TFT ---
  ecran.init();
  ecran.setRotation(1);
  ecran.fillScreen(TFT_BLACK);

  // --- Mémorisation temps de démarrage ---
  tempsInit = millis();

  Serial.println("[SETUP] Initialisation terminée");
}

// =============================================================
//  LOOP — Machine à états principale (non bloquante)
//  Appelée en boucle continue par l'ESP32
// =============================================================
void loop() {

  // --- Lecture capteurs cadencée (toutes les INTERVALLE_LECTURE ms) ---
  // Remplace l'appel direct à chaque itération qui saturait le bus I2C
  // et le port série avec des milliers de messages d'erreur SCD40
  if (millis() - derniereLecture >= INTERVALLE_LECTURE) {
    lireCapteurs();
    derniereLecture = millis();
  }

  // --- Affichage TFT uniquement lors d'un changement d'état ---
  // Évite les redessins inutiles qui ralentissent la boucle
  if (etat != etatPrecedent) {
    afficherEcran(etat);
    etatPrecedent = etat;
    // Réinitialiser le flag scan si on quitte CHECK_CAPTEURS
    if (etat != ETAT_CHECK_CAPTEURS) {
      scanEffectue = false;
    }
    Serial.printf("[ÉTAT] Transition → %d\n", etat);
  }

  // --- Dispatch selon état courant ---
  switch (etat) {

    // ----------------------------------------------------------
    //  ÉTAT 0 — INIT
    //  Durée fixe DUREE_INIT ms, puis passage automatique
    // ----------------------------------------------------------
    case ETAT_INIT:
      if (millis() - tempsInit >= DUREE_INIT) {
        Serial.println("[INIT] Terminé → CHECK_CAPTEURS");
        etat = ETAT_CHECK_CAPTEURS;
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 1 — CHECK CAPTEURS
    //
    //  Déroulement en deux phases :
    //
    //  Phase 1 — Scan automatique (une seule fois à l'entrée) :
    //    Scanne SHT35 et SCD40 (3 tentatives chacun)
    //    Affiche le résultat sur TFT et Serial
    //    Met à jour scanSHT35OK et scanSCD40OK
    //
    //  Phase 2 — Attente bouton GO :
    //    L'opérateur lit le résultat sur l'écran
    //    Appui GO :
    //      Si SHT35 OK → passe à ATTENTE_DONNEES
    //      Si SHT35 KO → passe en ERREUR (capteur principal manquant)
    //    SCD40 absent n'est pas bloquant (co2Actif reste false)
    // ----------------------------------------------------------
    case ETAT_CHECK_CAPTEURS:

      // Phase 1 : scan I2C (exécuté une seule fois à l'entrée de l'état)
      if (!scanEffectue) {
        verifierCapteursI2C();  // Scan + affichage TFT + mise à jour flags
        scanEffectue = true;
        Serial.println("[CHECK] Résultat affiché — appuyez GO pour continuer");
      }

      // Phase 2 : attente bouton GO pour valider et passer à la suite
      if (digitalRead(BROCHE_BOUTON_GO) == LOW) {
        delay(50);  // Anti-rebond
        if (digitalRead(BROCHE_BOUTON_GO) == LOW) {
          if (scanSHT35OK) {
            Serial.println("[CHECK] GO validé — SHT35 OK → ATTENTE_DONNEES");
            etat = ETAT_ATTENTE_DONNEES;
          } else {
            Serial.println("[CHECK] GO validé — SHT35 ABSENT → ERREUR");
            signalerErreur("SHT35 requis");
          }
        }
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 2 — ATTENTE DONNÉES
    //  Valide que le SHT35 retourne des valeurs plausibles
    //  CO₂ non validé (SCD40 désactivé, co2 = 5.0f fixe)
    // ----------------------------------------------------------
    case ETAT_ATTENTE_DONNEES:
      if (temperature > 10.0f && temperature < 45.0f && humidite > 0.0f) {
        Serial.println("[DONNÉES] Valides → BUFFER");
        etat = ETAT_BUFFER;
      } else {
        Serial.printf("[DONNÉES] Invalides — T=%.1f HR=%.1f\n",
                      temperature, humidite);
        signalerErreur("Donnees invalides");
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 3 — BUFFER
    //  Attente que l'opérateur ferme l'enceinte et appuie sur GO
    //  Rafraîchissement écran toutes les 2s pour afficher les mesures
    // ----------------------------------------------------------
    case ETAT_BUFFER:
      // Rafraîchissement périodique de l'écran (affiche les valeurs live)
      if (millis() - dernierEcran >= INTERVALLE_ECRAN) {
        afficherEcran(ETAT_BUFFER);
        dernierEcran = millis();
      }

      // Détection appui bouton GO avec anti-rebond (double lecture à 50ms)
      if (digitalRead(BROCHE_BOUTON_GO) == LOW) {
        delay(50);
        if (digitalRead(BROCHE_BOUTON_GO) == LOW) {
          Serial.println("[BUFFER] GO appuyé → MARCHE");
          // Initialisation timers régulation
          dernierCO2   = millis();
          dernierEcran = millis();
          etatCO2      = CO2_IDLE;
          etat         = ETAT_MARCHE;
        }
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 4 — MARCHE
    //  Régulation active température + humidité
    //  CO₂ désactivé (miseAJourCO2 retourne immédiatement)
    // ----------------------------------------------------------
    case ETAT_MARCHE:

      // Sécurité température absolue — priorité maximale
      if (temperature >= TEMP_SECURITE) {
        Serial.printf("[SÉCURITÉ] T=%.1f°C > %.1f°C → ERREUR\n",
                      temperature, TEMP_SECURITE);
        signalerErreur("TEMP CRITIQUE");
        break;
      }

      // Régulation thermique bang-bang (Peltier + ventilateur)
      regulerTemperature();

      // Régulation CO₂ — retourne immédiatement si co2Actif = false
      // Vanne reste fermée (V114 NC = sécurité par défaut)
      miseAJourCO2();

      // Monitoring humidité passive (alerte si enceinte ouverte)
      miseAJourHumidite();

      // Rafraîchissement écran périodique
      if (millis() - dernierEcran >= INTERVALLE_ECRAN) {
        afficherEcran(ETAT_MARCHE);
        dernierEcran = millis();
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 5 — ERREUR
    //  Tout éteint, LED rouge clignotante
    //  Seul un reset matériel permet de sortir de cet état
    // ----------------------------------------------------------
    case ETAT_ERREUR:
      toutEteindre();

      // Clignotement LED rouge non bloquant (période 500ms)
      {
        static unsigned long dernierBlink = 0;
        static bool          ledActive    = false;
        if (millis() - dernierBlink >= 500) {
          ledActive    = !ledActive;
          digitalWrite(BROCHE_LED_ROUGE, ledActive ? HIGH : LOW);
          dernierBlink = millis();
        }
      }
      break;
  }
}

// =============================================================
//  SHT35 — Démarrage via bibliothèque Adafruit
//
//  Initialise le capteur SHT35 à l'adresse I2C 0x44.
//  Met à jour le flag sht35Operationnel.
//  Doit être appelé après Wire.begin().
// =============================================================
void sht35Demarrer() {
  if (sht35.begin(ADRESSE_SHT35)) {
    sht35Operationnel = true;
    Serial.println("[SHT35] Initialisé OK");
  } else {
    sht35Operationnel = false;
    Serial.println("[SHT35] ERREUR — non détecté");
  }
}

// =============================================================
//  SCD40 — Envoi commande I2C brute
//  DÉSACTIVÉ — fonction conservée pour réactivation future
//
//  Envoie une commande 16 bits au SCD40 via le bus I2C.
//  Le SCD40 attend les commandes en big-endian (MSB en premier).
//
//  Paramètre :
//    cmd — commande 16 bits (ex. SCD40_CMD_START_MEAS = 0x21B1)
// =============================================================
void scd40EnvoyerCommande(uint16_t cmd) {
  // ⚠️ DÉSACTIVÉ — décommenter quand SCD40 branché
  // Wire.beginTransmission(ADRESSE_SCD40);
  // Wire.write((uint8_t)(cmd >> 8));    // Octet de poids fort (MSB)
  // Wire.write((uint8_t)(cmd & 0xFF));  // Octet de poids faible (LSB)
  // Wire.endTransmission();
}

// =============================================================
//  SCD40 — Démarrage mesure périodique
//  DÉSACTIVÉ — fonction conservée pour réactivation future
//
//  Pour réactiver :
//    1. Décommenter le corps de cette fonction
//    2. Décommenter scd40Demarrer() dans setup()
//    3. Passer co2Actif = true
//    4. Remettre co2 = 0.0f dans les variables globales
//    5. Décommenter le scan SCD40 dans verifierCapteursI2C()
// =============================================================
void scd40Demarrer() {
  // ⚠️ DÉSACTIVÉ — décommenter quand SCD40 branché
  // scd40EnvoyerCommande(SCD40_CMD_START_MEAS);
  // scd40Demarre = true;
  // Serial.println("[SCD40] Mesure périodique démarrée");
}

// =============================================================
//  SCD40 — Calcul CRC-8 (obligatoire pour validation données)
//  CONSERVÉ — utilisé dès que SCD40 réactivé
//
//  Le protocole SCD40 impose une vérification CRC-8 sur chaque
//  paire d'octets de données. Polynôme : 0x31, valeur init : 0xFF.
//
//  Paramètres :
//    data — pointeur vers le tableau d'octets à vérifier
//    len  — nombre d'octets à inclure dans le calcul
//
//  Retourne : CRC calculé (uint8_t)
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
//  SCD40 — Lecture mesure (I2C brute)
//  DÉSACTIVÉE — fonction conservée pour réactivation future
//
//  Pour réactiver : décommenter le corps de la fonction
//  et suivre les instructions de scd40Demarrer()
// =============================================================
bool scd40Lire(float& co2_out, float& temp_out, float& hum_out) {
  // ⚠️ DÉSACTIVÉ — décommenter quand SCD40 branché
  //
  // scd40EnvoyerCommande(SCD40_CMD_READ_MEAS);
  // delay(1);
  // Wire.requestFrom((uint8_t)ADRESSE_SCD40, (uint8_t)9);
  // if (Wire.available() < 9) {
  //   Serial.println("[SCD40] Erreur — données insuffisantes");
  //   return false;
  // }
  // uint8_t data[9];
  // for (uint8_t i = 0; i < 9; i++) { data[i] = Wire.read(); }
  // if (calculerCRC(data,     2) != data[2]) { return false; }
  // if (calculerCRC(data + 3, 2) != data[5]) { return false; }
  // if (calculerCRC(data + 6, 2) != data[8]) { return false; }
  // uint16_t raw_co2  = ((uint16_t)data[0] << 8) | data[1];
  // uint16_t raw_temp = ((uint16_t)data[3] << 8) | data[4];
  // uint16_t raw_hum  = ((uint16_t)data[6] << 8) | data[7];
  // co2_out  = (float)raw_co2;
  // temp_out = -45.0f + 175.0f * (raw_temp / 65535.0f);
  // hum_out  = 100.0f * (raw_hum  / 65535.0f);
  // Serial.printf("[SCD40] CO₂=%.0fppm  T=%.1f°C  HR=%.1f%%\n", co2_out, temp_out, hum_out);
  // return true;

  return false;  // Retourne toujours false tant que désactivé
}

// =============================================================
//  LECTURE CAPTEURS — Point d'entrée unique
//
//  Appelée toutes les INTERVALLE_LECTURE ms (2s) depuis loop()
//  via le timer derniereLecture — évite la saturation du bus I2C
//
//  Active : SHT35 (température + humidité)
//  Inactif : SCD40 (co2 reste à valeur fixe 5.0f)
//
//  Pour réactiver SCD40 : décommenter le bloc marqué ⚠️
// =============================================================
void lireCapteurs() {

  // --- SHT35 — Température et humidité ---
  if (sht35Operationnel) {
    float t = sht35.readTemperature();
    float h = sht35.readHumidity();

    // isnan() détecte une erreur de lecture Adafruit (retourne NaN si échec)
    if (!isnan(t)) temperature = t;
    if (!isnan(h)) humidite    = h;

    Serial.printf("[SHT35] T=%.1f°C  HR=%.1f%%\n", temperature, humidite);
  }

  // --- SCD40 — CO₂ : DÉSACTIVÉ ---
  // Pour réactiver : décommenter le bloc suivant
  // ET passer co2Actif = true dans les variables globales
  //
  // ⚠️ DÉSACTIVÉ
  // if (scd40Demarre) {
  //   float co2_ppm, temp_scd, hum_scd;
  //   if (scd40Lire(co2_ppm, temp_scd, hum_scd)) {
  //     co2 = co2_ppm / 10000.0f;  // Conversion ppm → %
  //   }
  // }
}

// =============================================================
//  SCAN I2C — Tentatives multiples sur une adresse
//
//  Tente de contacter un périphérique I2C à l'adresse donnée.
//  Effectue jusqu'à 'tentatives' essais avec un délai entre chaque.
//
//  Paramètres :
//    adresse    — adresse I2C 7 bits du périphérique
//    tentatives — nombre maximum de tentatives (recommandé : 3)
//
//  Retourne : true si le périphérique répond (ACK reçu), false sinon
//
//  Codes retour Wire.endTransmission() :
//    0 → succès (ACK reçu, périphérique présent)
//    2 → NACK adresse (périphérique absent)
//    3 → NACK données
//    4 → autre erreur bus
// =============================================================
bool scannerAdresse(uint8_t adresse, uint8_t tentatives) {
  for (uint8_t i = 0; i < tentatives; i++) {
    Wire.beginTransmission(adresse);
    uint8_t erreur = Wire.endTransmission();

    if (erreur == 0) {
      return true;  // Périphérique détecté
    }

    Serial.printf("[I2C] Tentative %d/%d addr=0x%02X erreur=%d\n",
                  i + 1, tentatives, adresse, erreur);

    if (i < tentatives - 1) {
      delay(DELAI_RETRY_I2C);  // Attente avant prochaine tentative
    }
  }
  return false;  // Toutes les tentatives ont échoué
}

// =============================================================
//  VÉRIFICATION CAPTEURS I2C
//
//  Scanne SHT35 (obligatoire) et SCD40 (optionnel, désactivé).
//  Met à jour les flags globaux scanSHT35OK et scanSCD40OK.
//  Affiche le résultat sur TFT via afficherResultatI2C().
//
//  Le passage à l'état suivant N'EST PAS décidé ici —
//  c'est le bouton GO dans ETAT_CHECK_CAPTEURS qui valide.
//
//  SCD40 : scan commenté → scanSCD40OK = false systématiquement
//  Pour réactiver le scan SCD40 : décommenter le bloc marqué ⚠️
// =============================================================
void verifierCapteursI2C() {

  Serial.println("[I2C] Scan en cours...");

  // --- Scan SHT35 (obligatoire) ---
  scanSHT35OK = scannerAdresse(ADRESSE_SHT35, TENTATIVES_I2C);
  Serial.printf("[I2C] SHT35 [0x%02X] → %s\n",
                ADRESSE_SHT35, scanSHT35OK ? "OK" : "ABSENT");

  // --- Scan SCD40 : DÉSACTIVÉ ---
  // Pour réactiver : décommenter les 3 lignes suivantes
  // ET passer co2Actif = true si détecté
  //
  // ⚠️ DÉSACTIVÉ
  // scanSCD40OK = scannerAdresse(ADRESSE_SCD40, TENTATIVES_I2C);
  // Serial.printf("[I2C] SCD40 [0x%02X] → %s\n",
  //               ADRESSE_SCD40, scanSCD40OK ? "OK" : "ABSENT");
  // if (scanSCD40OK) co2Actif = true;

  scanSCD40OK = false;  // Toujours false tant que SCD40 désactivé

  // Met à jour sht35Operationnel pour lireCapteurs()
  sht35Operationnel = scanSHT35OK;

  // Affichage résultat sur TFT
  afficherResultatI2C(scanSHT35OK, scanSCD40OK);
}

// =============================================================
//  RÉGULATION THERMIQUE — Bang-bang avec zone morte
//
//  Contrôle le Peltier TEC1-12703 (chauffage seul, MOSFET Grove)
//  et le ventilateur selon la température mesurée par le SHT35.
//
//  Logique :
//    T < SEUIL_CHAUFFE   (36.5°C) → Peltier ON  + Ventilateur OFF
//    T > SEUIL_REFROIDIT (37.5°C) → Peltier OFF + Ventilateur ON
//    Zone neutre [36.5 - 37.5°C]  → Peltier OFF + Ventilateur OFF
//
//  Note : pas de refroidissement actif (MOSFET = 1 sens uniquement)
//  Le ventilateur en mode "trop chaud" assure une dissipation passive.
//
//  Appelée en continu dans ETAT_MARCHE.
//  La sécurité T > TEMP_SECURITE est gérée dans loop() avant appel.
// =============================================================
void regulerTemperature() {

  if (temperature < SEUIL_CHAUFFE) {
    // --- Chauffe ---
    digitalWrite(BROCHE_PELTIER,     HIGH);
    digitalWrite(BROCHE_VENTILATEUR, LOW);
    Serial.printf("[THERM] Chauffe — T=%.1f°C\n", temperature);

  } else if (temperature > SEUIL_REFROIDIT) {
    // --- Trop chaud — dissipation passive ventilateur ---
    digitalWrite(BROCHE_PELTIER,     LOW);
    digitalWrite(BROCHE_VENTILATEUR, HIGH);
    Serial.printf("[THERM] Trop chaud — T=%.1f°C\n", temperature);

  } else {
    // --- Zone neutre [36.5 - 37.5°C] ---
    digitalWrite(BROCHE_PELTIER,     LOW);
    digitalWrite(BROCHE_VENTILATEUR, LOW);
  }
}

// =============================================================
//  RÉGULATION CO₂ — Injection impulsionnelle non bloquante
//  DÉSACTIVÉE via flag co2Actif = false
//
//  Retourne immédiatement si co2Actif = false.
//  La vanne reste fermée (V114 NC = sécurité par défaut).
//
//  Pour réactiver :
//    1. Passer co2Actif = true
//    2. Brancher et activer SCD40 (voir scd40Demarrer)
//
//  Sous-machine CO2_IDLE → CO2_INJECTING → CO2_STABILIZING
//  conservée intacte pour réactivation sans réécriture.
// =============================================================
void miseAJourCO2() {

  // ⚠️ DÉSACTIVÉ — retour immédiat tant que SCD40 non branché
  // La vanne reste LOW (V114 NC fermée = aucune injection)
  if (!co2Actif) return;

  unsigned long maintenant = millis();

  switch (etatCO2) {

    case CO2_IDLE:
      if (maintenant - dernierCO2 >= INTERVALLE_CO2) {
        dernierCO2 = maintenant;
        Serial.printf("[CO₂] Lecture : %.2f%%\n", co2);

        if (co2 < SEUIL_BAS_CO2) {
          digitalWrite(BROCHE_SOLENOIDE, HIGH);
          timerCO2 = maintenant;
          etatCO2  = CO2_INJECTING;
          Serial.printf("[CO₂] Injection déclenchée (%.2f%% < %.2f%%)\n",
                        co2, SEUIL_BAS_CO2);
        } else if (co2 > SEUIL_HAUT_CO2) {
          Serial.printf("[CO₂] Trop élevé — dilution naturelle (%.2f%%)\n", co2);
        }
      }
      break;

    case CO2_INJECTING:
      if (maintenant - timerCO2 >= DUREE_INJECT_MS) {
        digitalWrite(BROCHE_SOLENOIDE, LOW);
        timerCO2 = maintenant;
        etatCO2  = CO2_STABILIZING;
        Serial.println("[CO₂] Vanne fermée → stabilisation 30s");
      }
      break;

    case CO2_STABILIZING:
      if (maintenant - timerCO2 >= DUREE_STABILISATION) {
        etatCO2 = CO2_IDLE;
        Serial.println("[CO₂] Stabilisation terminée → IDLE");
      }
      break;
  }
}

// =============================================================
//  MONITORING HUMIDITÉ — Passif avec alerte
//
//  L'humidité n'est pas régulée activement (éponge saturée
//  au départ + enceinte fermée = système quasi-fermé sur 1h).
//
//  Cette fonction vérifie uniquement si la HR chute sous
//  SEUIL_HR_ALERTE (85%). Une chute sous ce seuil signale
//  un problème d'étanchéité de l'enceinte.
//
//  En cas d'alerte :
//    - Flag alerteHR mis à true
//    - Message Serial
//    - Affiché sur TFT dans l'état MARCHE
//
//  Aucun actionneur humidité dans ce firmware.
//  Appelée en continu dans ETAT_MARCHE.
// =============================================================
void miseAJourHumidite() {
  if (humidite < SEUIL_HR_ALERTE) {
    if (!alerteHR) {
      alerteHR = true;
      Serial.printf("[HR] ALERTE — HR=%.1f%% < %.1f%% — enceinte ouverte ?\n",
                    humidite, SEUIL_HR_ALERTE);
    }
  } else {
    if (alerteHR) {
      alerteHR = false;
      Serial.printf("[HR] Alerte levée — HR=%.1f%%\n", humidite);
    }
  }
}

// =============================================================
//  TOUT ÉTEINDRE — Mise en sécurité complète
//
//  Éteint tous les actionneurs dans un état sûr :
//    - Peltier  : OFF
//    - Ventilateur : OFF
//    - Électrovanne V114 : LOW (NC fermée = sécurité)
//    - LED rouge : OFF
//
//  Appelée dans setup(), signalerErreur() et ETAT_ERREUR.
// =============================================================
void toutEteindre() {
  digitalWrite(BROCHE_PELTIER,     LOW);
  digitalWrite(BROCHE_VENTILATEUR, LOW);
  digitalWrite(BROCHE_SOLENOIDE,   LOW);  // V114 NC → fermée = sécurité
  digitalWrite(BROCHE_LED_ROUGE,   LOW);
}

// =============================================================
//  SIGNALER ERREUR — Passage forcé en ETAT_ERREUR
//
//  Éteint tous les actionneurs, affiche le message d'erreur
//  sur le TFT et sur le port série, puis force l'état ERREUR.
//
//  Paramètre :
//    message — chaîne C décrivant la cause de l'erreur
//
//  Depuis ETAT_ERREUR, seul un reset matériel permet de redémarrer.
// =============================================================
void signalerErreur(const char* message) {
  Serial.printf("[ERREUR] %s\n", message);
  toutEteindre();
  etat = ETAT_ERREUR;
  afficherEcran(ETAT_ERREUR);
}

// =============================================================
//  AFFICHAGE RÉSULTAT I2C — Écran dédié CHECK_CAPTEURS
//
//  Affiche l'état de détection de chaque capteur I2C :
//    - SHT35 [0x44] : OK (vert) ou ABSENT (rouge)
//    - SCD40 [0x62] : DESACTIVE (gris) ou OK (vert) si réactivé
//    - Résultat global + instruction "Appuyez GO"
//
//  Paramètres :
//    sht35 — true si SHT35 détecté
//    scd40 — true si SCD40 détecté (toujours false actuellement)
// =============================================================
void afficherResultatI2C(bool sht35, bool scd40) {
  ecran.fillScreen(0x1E9F);
  ecran.setTextDatum(MC_DATUM);

  ecran.setTextColor(TFT_WHITE, 0x1E9F);
  ecran.setTextSize(2);
  ecran.drawString("Capteurs I2C", 120, 30);

  ecran.setTextSize(1);

  // SHT35
  ecran.setTextColor(sht35 ? TFT_GREEN : TFT_RED, 0x1E9F);
  ecran.drawString(sht35 ? "SHT35  [0x44]  OK"
                         : "SHT35  [0x44]  ABSENT",
                   120, 85);

  // SCD40 — affiché comme désactivé tant que commenté
  ecran.setTextColor(scd40 ? TFT_GREEN : 0xC618, 0x1E9F);  // 0xC618 = gris
  ecran.drawString(scd40 ? "SCD40  [0x62]  OK"
                         : "SCD40  [0x62]  DESACTIVE",
                   120, 110);

  // Résultat global
  ecran.setTextSize(2);
  if (sht35) {
    ecran.setTextColor(TFT_GREEN, 0x1E9F);
    ecran.drawString("SHT35 OK", 120, 155);
  } else {
    ecran.setTextColor(TFT_RED, 0x1E9F);
    ecran.drawString("SHT35 ABSENT", 120, 155);
  }

  // Instruction bouton
  ecran.setTextSize(1);
  ecran.setTextColor(TFT_WHITE, 0x1E9F);
  ecran.drawString("Appuyez GO pour continuer", 120, 200);
}

// =============================================================
//  AFFICHAGE ÉCRAN TFT — Dispatcher principal
//
//  Redessine l'écran complet selon l'état courant.
//  Chaque état a sa couleur de fond distinctive.
//
//  Couleurs par état :
//    INIT            : bleu marine  (TFT_NAVY)
//    CHECK_CAPTEURS  : bleu-vert    (0x1E9F)
//    ATTENTE_DONNÉES : bleu-vert    (0x1E9F)
//    BUFFER          : orange       (0xFD20)
//    MARCHE          : vert         (0x07E0)
//    ERREUR          : rouge        (TFT_RED)
// =============================================================
void afficherEcran(int e) {
  ecran.fillScreen(TFT_BLACK);
  ecran.setTextDatum(MC_DATUM);

  switch (e) {

    case ETAT_INIT:
      ecran.fillScreen(TFT_NAVY);
      ecran.setTextColor(TFT_WHITE, TFT_NAVY);
      ecran.setTextSize(2);
      ecran.drawString("PB-0326", 120, 50);
      ecran.setTextSize(1);
      ecran.drawString("Incubateur transportable", 120, 95);
      ecran.drawString("HEPIA HES-SO  Groupe 3", 120, 120);
      ecran.drawString("Initialisation...", 120, 160);
      break;

    case ETAT_CHECK_CAPTEURS:
      // Cet écran est géré par afficherResultatI2C()
      // appelé depuis verifierCapteursI2C()
      // Affichage générique pendant le scan
      ecran.fillScreen(0x1E9F);
      ecran.setTextColor(TFT_WHITE, 0x1E9F);
      ecran.setTextSize(2);
      ecran.drawString("Scan I2C...", 120, 100);
      break;

    case ETAT_ATTENTE_DONNEES:
      ecran.fillScreen(0x1E9F);
      ecran.setTextColor(TFT_WHITE, 0x1E9F);
      ecran.setTextSize(2);
      ecran.drawString("Verification", 120, 80);
      ecran.setTextSize(1);
      ecran.drawString("Lecture capteurs...", 120, 140);
      ecran.drawString("Validation plages", 120, 165);
      break;

    case ETAT_BUFFER:
      {
        ecran.fillScreen(0xFD20);
        ecran.setTextColor(TFT_BLACK, 0xFD20);
        ecran.setTextSize(2);
        ecran.drawString("PRET", 120, 30);
        ecran.setTextSize(1);
        char buf[32];
        snprintf(buf, sizeof(buf), "T:   %.1f C", temperature);
        ecran.drawString(buf, 120, 90);
        snprintf(buf, sizeof(buf), "HR:  %.1f %%", humidite);
        ecran.drawString(buf, 120, 115);
        snprintf(buf, sizeof(buf), "CO2: %.2f %% (fixe)", co2);
        ecran.drawString(buf, 120, 140);
        ecran.setTextColor(0x7BEF, 0xFD20);  // Gris pour indiquer CO2 inactif
        ecran.drawString("CO2 desactive", 120, 160);
        ecran.setTextColor(TFT_BLACK, 0xFD20);
        ecran.drawString("Fermez l'enceinte", 120, 190);
        ecran.drawString("puis appuyez GO", 120, 210);
        break;
      }

    case ETAT_MARCHE:
      {
        ecran.fillScreen(0x07E0);
        ecran.setTextColor(TFT_BLACK, 0x07E0);
        ecran.setTextSize(2);
        ecran.drawString("EN MARCHE", 120, 20);
        ecran.setTextSize(1);
        char ligne[32];

        // Température avec indicateur [+]/[-]/[=]
        const char* indT = (temperature < SEUIL_CHAUFFE)   ? " [+]" :
                           (temperature > SEUIL_REFROIDIT)  ? " [-]" : " [=]";
        snprintf(ligne, sizeof(ligne), "T: %.1f/%.1f C%s",
                 temperature, CONSIGNE_TEMP, indT);
        ecran.drawString(ligne, 120, 75);

        // Humidité avec alerte
        if (alerteHR) {
          ecran.setTextColor(TFT_RED, 0x07E0);
          snprintf(ligne, sizeof(ligne), "HR: %.1f %% ALERTE", humidite);
        } else {
          snprintf(ligne, sizeof(ligne), "HR: %.1f %%", humidite);
        }
        ecran.drawString(ligne, 120, 105);
        ecran.setTextColor(TFT_BLACK, 0x07E0);

        // CO₂ — indiqué comme désactivé
        ecran.setTextColor(0x39E7, 0x07E0);  // Vert foncé pour indiquer inactif
        snprintf(ligne, sizeof(ligne), "CO2: %.2f %% [OFF]", co2);
        ecran.drawString(ligne, 120, 135);
        ecran.setTextColor(TFT_BLACK, 0x07E0);

        // État Peltier
        const char* etatPeltier = (temperature < SEUIL_CHAUFFE)   ? "Chauffe" :
                                  (temperature > SEUIL_REFROIDIT)  ? "Dissipe" : "Neutre";
        snprintf(ligne, sizeof(ligne), "Peltier: %s", etatPeltier);
        ecran.drawString(ligne, 120, 165);
        break;
      }

    case ETAT_ERREUR:
      ecran.fillScreen(TFT_RED);
      ecran.setTextColor(TFT_WHITE, TFT_RED);
      ecran.setTextSize(2);
      ecran.drawString("ERREUR", 120, 80);
      ecran.setTextSize(1);
      ecran.drawString("Systeme arrete", 120, 135);
      ecran.drawString("Reset requis", 120, 160);
      break;
  }
}
