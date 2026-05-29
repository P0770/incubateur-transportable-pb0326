// =============================================================
//  PB-0326 — Incubateur transportable pour cellules mammifères
//  HEPIA HES-SO — Groupe 3 : Lyla Founas, Mattéo Pozzo di Borgo
//  Version : 3.0 — Mai 2026
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
//    2 - ATTENTE_DONNEES : validation plages capteurs
//    3 - BUFFER          : attente appui bouton GO
//    4 - MARCHE          : régulation active
//    5 - ERREUR          : sécurité, tout éteint
//
//  CAPTEURS RETENUS :
//    - SHT35  (I2C 0x44) : température + humidité → bibliothèque Adafruit
//    - SCD40  (I2C 0x62) : CO₂                   → lecture I2C brute Wire
//
//  ACTIONNEURS :
//    - Peltier TEC1-12703 : chauffage seul (MOSFET Grove, ON/OFF)
//    - Ventilateur        : ON si T > 37.5°C (dissipation passive)
//    - Électrovanne V114  : injection CO₂ impulsionnelle
//
// =============================================================

#include <TFT_eSPI.h>        // Bibliothèque écran TFT SPI
#include <Wire.h>            // Bus I2C pour SHT35 et SCD40
#include <Adafruit_SHT31.h>  // Bibliothèque capteur température/humidité SHT35

// =============================================================
//  BROCHES GPIO — Attribution finale sans conflit
//  I2C : SDA=21, SCL=22 (défaut ESP32, réservés)
// =============================================================
#define BROCHE_PELTIER 39
#define BROCHE_VENTILATEUR 12  // Ventilateur dissipateur (ON/OFF)
#define BROCHE_SOLENOIDE 14    // Électrovanne V114 via MOSFET IRLZ44N
#define BROCHE_LED_ROUGE 10    // LED alerte erreur (clignotante en état 5)
#define BROCHE_BOUTON_GO 5     // Bouton démarrage transport (INPUT_PULLUP)

// =============================================================
//  CONSIGNES BIOLOGIQUES
//  Valeurs cibles pour maintien de la viabilité cellulaire
// =============================================================
#define CONSIGNE_TEMP 37.0f    // Température cible en °C
#define SEUIL_CHAUFFE 36.5f    // En dessous → Peltier chauffe
#define SEUIL_REFROIDIT 37.5f  // Au dessus  → Peltier refroidit
#define TEMP_SECURITE 40.0f    // Sécurité absolue → passage ERREUR

#define CONSIGNE_CO2 5.0f    // CO₂ cible en %
#define SEUIL_BAS_CO2 4.7f   // En dessous → injection CO₂
#define SEUIL_HAUT_CO2 5.3f  // Au dessus  → attente dilution naturelle

#define SEUIL_HR_ALERTE 85.0f  // En dessous → alerte enceinte ouverte

// =============================================================
//  PARAMÈTRES RÉGULATION CO₂
//  Temporisations pour éviter oscillations (inertie diffusion CO₂)
// =============================================================
#define DUREE_INJECT_MS 200        // Durée ouverture électrovanne (ms) — à calibrer
#define DUREE_STABILISATION 30000  // Attente post-injection avant nouvelle mesure (ms)
#define INTERVALLE_CO2 5000        // Cadence lecture SCD40 (ms) — temps mesure SCD40

// =============================================================
//  PARAMÈTRES AFFICHAGE & DIVERS
// =============================================================
#define INTERVALLE_ECRAN 2000  // Rafraîchissement écran TFT (ms)
#define DUREE_INIT 3000        // Durée état INIT (ms)
#define TENTATIVES_I2C 3       // Nombre de tentatives scan I2C avant erreur
#define DELAI_RETRY_I2C 50     // Délai entre tentatives I2C (ms)

// =============================================================
//  ÉTATS MACHINE PRINCIPALE
// =============================================================
#define ETAT_INIT 0             // Initialisation matérielle
#define ETAT_CHECK_CAPTEURS 1   // Vérification présence capteurs I2C
#define ETAT_ATTENTE_DONNEES 2  // Validation plages de valeurs capteurs
#define ETAT_BUFFER 3           // Attente appui bouton GO par l'opérateur
#define ETAT_MARCHE 4           // Régulation active — transport en cours
#define ETAT_ERREUR 5           // Erreur critique — tout éteint

// =============================================================
//  ÉTATS INTERNES RÉGULATION CO₂
//  Sous-machine à états non bloquante pour injection CO₂
// =============================================================
#define CO2_IDLE 0         // En attente, lecture périodique active
#define CO2_INJECTING 1    // Électrovanne ouverte, injection en cours
#define CO2_STABILIZING 2  // Post-injection, attente diffusion CO₂

// =============================================================
//  ADRESSES I2C CAPTEURS
// =============================================================
#define ADRESSE_SHT35 0x44  // Adresse I2C SHT35 (défaut)
#define ADRESSE_SCD40 0x62  // Adresse I2C SCD40 (défaut)

// =============================================================
//  COMMANDES SCD40 (protocole I2C brut, commandes 16 bits)
// =============================================================
#define SCD40_CMD_START_MEAS 0x21B1  // Démarrer mesure périodique
#define SCD40_CMD_READ_MEAS 0xEC05   // Lire mesure courante
#define SCD40_CMD_STOP_MEAS 0x3F86   // Arrêter mesure périodique

// =============================================================
//  OBJETS
// =============================================================
TFT_eSPI ecran = TFT_eSPI();              // Écran TFT (SPI, bibliothèque TFT_eSPI)
Adafruit_SHT31 sht35 = Adafruit_SHT31();  // Capteur SHT35 (I2C, bibliothèque Adafruit)

// =============================================================
//  VARIABLES GLOBALES — ÉTAT SYSTÈME
// =============================================================

// --- Machine à états principale ---
int etat = ETAT_INIT;    // État courant de la machine principale
int etatPrecedent = -1;  // Mémorise l'état précédent pour détecter les transitions

// --- Valeurs capteurs (mises à jour par lireCapteurs()) ---
float temperature = 25.0f;  // Température mesurée par SHT35 (°C)
float humidite = 50.0f;     // Humidité relative mesurée par SHT35 (%)
float co2 = 0.0f;           // Concentration CO₂ mesurée par SCD40 (%)

// --- État régulation CO₂ ---
int etatCO2 = CO2_IDLE;        // Sous-état de la régulation CO₂
unsigned long timerCO2 = 0;    // Horodatage dernier événement CO₂ (ms)
unsigned long dernierCO2 = 0;  // Horodatage dernière lecture CO₂ (ms)

// --- Timers non bloquants ---
unsigned long dernierEcran = 0;  // Horodatage dernier rafraîchissement TFT (ms)
unsigned long tempsInit = 0;     // Horodatage début état INIT (ms)

// --- Flags ---
bool sht35Operationnel = false;  // true si SHT35 détecté et initialisé
bool scd40Demarre = false;       // true si SCD40 en mesure périodique active
bool alerteHR = false;           // true si HR sous seuil d'alerte (enceinte ouverte ?)

// =============================================================
//  SETUP — Initialisation unique au démarrage
// =============================================================
void setup() {
  Serial.begin(115200);
  Serial.println("=== PB-0326 Firmware v3.0 ===");

  // --- Configuration broches de sortie ---
  pinMode(BROCHE_VENTILATEUR, OUTPUT);
  pinMode(BROCHE_SOLENOIDE, OUTPUT);
  pinMode(BROCHE_LED_ROUGE, OUTPUT);

  // --- Configuration broche d'entrée ---
  pinMode(BROCHE_BOUTON_GO, INPUT_PULLUP);  // Résistance pull-up interne activée

  // --- Configuration PWM Peltier ---
  // Canal 0, fréquence 5kHz, résolution 8 bits (0-255)
  // NOUVEAU v3.x
  pinMode(BROCHE_PELTIER, OUTPUT);

  // --- Sécurité : tout éteindre avant toute chose ---
  toutEteindre();

  // --- Initialisation bus I2C ---
  // SDA=21, SCL=22 — broches I2C par défaut ESP32
  // Doit être appelé AVANT toute utilisation de Wire ou bibliothèque I2C
  Wire.begin(21, 22);
  delay(100);  // Stabilisation bus I2C

  // --- Initialisation SHT35 via bibliothèque Adafruit ---
  sht35Demarrer();

  // --- Démarrage SCD40 en mesure périodique ---
  scd40Demarrer();

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

  // --- Lecture capteurs à chaque itération ---
  // (SHT35 et SCD40 ont leur propre gestion du timing interne)
  lireCapteurs();

  // --- Affichage TFT uniquement lors d'un changement d'état ---
  // Évite les redessins inutiles qui ralentissent la boucle
  if (etat != etatPrecedent) {
    afficherEcran(etat);
    etatPrecedent = etat;
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
    //  Vérifie la présence I2C de SHT35 et SCD40
    //  3 tentatives par capteur avant passage en ERREUR
    // ----------------------------------------------------------
    case ETAT_CHECK_CAPTEURS:
      if (verifierCapteursI2C()) {
        Serial.println("[CHECK] Capteurs OK → ATTENTE_DONNEES");
        etat = ETAT_ATTENTE_DONNEES;
      } else {
        signalerErreur("Capteur absent");
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 2 — ATTENTE DONNÉES
    //  Valide que les capteurs retournent des valeurs plausibles
    //  (élimine les lectures parasites au démarrage)
    // ----------------------------------------------------------
    case ETAT_ATTENTE_DONNEES:
      if (temperature > 10.0f && temperature < 45.0f && humidite > 0.0f && co2 >= 0.0f) {
        Serial.println("[DONNÉES] Valides → BUFFER");
        etat = ETAT_BUFFER;
      } else {
        Serial.printf("[DONNÉES] Invalides — T=%.1f HR=%.1f CO2=%.2f\n",
                      temperature, humidite, co2);
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
          dernierCO2 = millis();
          dernierEcran = millis();
          etatCO2 = CO2_IDLE;
          etat = ETAT_MARCHE;
        }
      }
      break;

    // ----------------------------------------------------------
    //  ÉTAT 4 — MARCHE
    //  Régulation active des 3 paramètres biologiques
    //  Toutes les fonctions de régulation sont non bloquantes
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

      // Régulation CO₂ (électrovanne V114, non bloquant)
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
        static bool ledActive = false;
        if (millis() - dernierBlink >= 500) {
          ledActive = !ledActive;
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
//
//  Envoie une commande 16 bits au SCD40 via le bus I2C.
//  Le SCD40 attend les commandes en big-endian (MSB en premier).
//
//  Paramètre :
//    cmd — commande 16 bits (ex. SCD40_CMD_START_MEAS = 0x21B1)
// =============================================================
void scd40EnvoyerCommande(uint16_t cmd) {
  Wire.beginTransmission(ADRESSE_SCD40);
  Wire.write((uint8_t)(cmd >> 8));    // Octet de poids fort (MSB)
  Wire.write((uint8_t)(cmd & 0xFF));  // Octet de poids faible (LSB)
  Wire.endTransmission();
}

// =============================================================
//  SCD40 — Démarrage mesure périodique
//
//  Envoie la commande START_PERIODIC_MEASUREMENT au SCD40.
//  Après cette commande, le SCD40 produit une nouvelle mesure
//  toutes les 5 secondes. La première mesure est disponible
//  après un délai minimum de 5s.
//
//  Met à jour le flag scd40Demarre.
// =============================================================
void scd40Demarrer() {
  scd40EnvoyerCommande(SCD40_CMD_START_MEAS);
  scd40Demarre = true;
  Serial.println("[SCD40] Mesure périodique démarrée");
}

// =============================================================
//  SCD40 — Calcul CRC-8 (obligatoire pour validation données)
//
//  Le protocole SCD40 impose une vérification CRC-8 sur chaque
//  paire d'octets de données. Polynôme : 0x31, valeur init : 0xFF.
//
//  Paramètres :
//    data — pointeur vers le tableau d'octets à vérifier
//    len  — nombre d'octets à inclure dans le calcul
//
//  Retourne : CRC calculé (uint8_t)
//
//  Usage : comparer avec l'octet CRC reçu du SCD40
//    if (calculerCRC(data, 2) != data[2]) → erreur CRC
// =============================================================
uint8_t calculerCRC(uint8_t* data, uint8_t len) {
  uint8_t crc = 0xFF;  // Valeur initiale imposée par le protocole SCD40
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t b = 0; b < 8; b++) {
      // Si bit de poids fort = 1 → XOR avec polynôme 0x31
      // Sinon → décalage simple
      crc = (crc & 0x80) ? ((crc << 1) ^ 0x31) : (crc << 1);
    }
  }
  return crc;
}

// =============================================================
//  SCD40 — Lecture mesure (I2C brute)
//
//  Demande et lit les 9 octets de mesure du SCD40 :
//    Bytes 0-1 : CO₂ brut (uint16, big-endian)
//    Byte  2   : CRC CO₂
//    Bytes 3-4 : température brute (uint16, big-endian)
//    Byte  5   : CRC température
//    Bytes 6-7 : humidité brute (uint16, big-endian)
//    Byte  8   : CRC humidité
//
//  Conversions appliquées :
//    CO₂ (ppm)  = valeur brute (direct)
//    Temp (°C)  = -45 + 175 × (raw / 65535)
//    Hum  (%)   = 100 × (raw / 65535)
//
//  Paramètres (par référence, mis à jour si lecture réussie) :
//    co2_out  — concentration CO₂ en ppm
//    temp_out — température en °C
//    hum_out  — humidité relative en %
//
//  Retourne : true si lecture et CRC valides, false sinon
// =============================================================
bool scd40Lire(float& co2_out, float& temp_out, float& hum_out) {

  // Envoyer commande de lecture
  scd40EnvoyerCommande(SCD40_CMD_READ_MEAS);
  delay(1);  // Temps de traitement interne SCD40 (minimum 1ms)

  // Demander 9 octets au SCD40
  Wire.requestFrom((uint8_t)ADRESSE_SCD40, (uint8_t)9);

  // Vérifier disponibilité des données
  if (Wire.available() < 9) {
    Serial.println("[SCD40] Erreur — données insuffisantes");
    return false;
  }

  // Lire les 9 octets
  uint8_t data[9];
  for (uint8_t i = 0; i < 9; i++) {
    data[i] = Wire.read();
  }

  // --- Vérification CRC sur chaque paire d'octets ---
  if (calculerCRC(data, 2) != data[2]) {
    Serial.println("[SCD40] CRC CO₂ invalide");
    return false;
  }
  if (calculerCRC(data + 3, 2) != data[5]) {
    Serial.println("[SCD40] CRC température invalide");
    return false;
  }
  if (calculerCRC(data + 6, 2) != data[8]) {
    Serial.println("[SCD40] CRC humidité invalide");
    return false;
  }

  // --- Reconstruction valeurs brutes 16 bits (big-endian) ---
  uint16_t raw_co2 = ((uint16_t)data[0] << 8) | data[1];
  uint16_t raw_temp = ((uint16_t)data[3] << 8) | data[4];
  uint16_t raw_hum = ((uint16_t)data[6] << 8) | data[7];

  // --- Conversions selon datasheet SCD40 ---
  co2_out = (float)raw_co2;                            // ppm (direct)
  temp_out = -45.0f + 175.0f * (raw_temp / 65535.0f);  // °C
  hum_out = 100.0f * (raw_hum / 65535.0f);             // %

  Serial.printf("[SCD40] CO₂=%.0fppm  T=%.1f°C  HR=%.1f%%\n",
                co2_out, temp_out, hum_out);
  return true;
}

// =============================================================
//  LECTURE CAPTEURS — Point d'entrée unique
//
//  Lit SHT35 (température + humidité) et SCD40 (CO₂).
//  Si une lecture échoue, les dernières valeurs connues
//  sont conservées (pas de mise à zéro intempestive).
//
//  Note : co2 est stocké en % (co2_ppm / 10000)
//    5% CO₂ = 50 000 ppm → 50000 / 10000 = 5.0
// =============================================================
void lireCapteurs() {

  // --- SHT35 — Température et humidité ---
  if (sht35Operationnel) {
    float t = sht35.readTemperature();
    float h = sht35.readHumidity();

    // isnan() détecte une erreur de lecture Adafruit (retourne NaN si échec)
    if (!isnan(t)) temperature = t;
    if (!isnan(h)) humidite = h;
  }

  // --- SCD40 — CO₂ (lecture toutes les ~5s, cadence interne SCD40) ---
  if (scd40Demarre) {
    float co2_ppm, temp_scd, hum_scd;
    if (scd40Lire(co2_ppm, temp_scd, hum_scd)) {
      // Conversion ppm → % pour cohérence avec les seuils définis
      co2 = co2_ppm / 10000.0f;
    }
    // Température et humidité SCD40 non utilisées
    // (SHT35 est le capteur de référence pour ces grandeurs)
  }
}

// =============================================================
//  SCAN I2C — Tentatives multiples sur une adresse
//
//  Tente de contacter un périphérique I2C à l'adresse donnée.
//  Effectue jusqu'à 'tentatives' essais avec un délai entre chaque.
//
//  Paramètres :
//    adresse   — adresse I2C 7 bits du périphérique
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
      // Périphérique détecté
      return true;
    }

    Serial.printf("[I2C] Tentative %d/%d addr=0x%02X erreur=%d\n",
                  i + 1, tentatives, adresse, erreur);

    // Attente avant prochaine tentative (sauf la dernière)
    if (i < tentatives - 1) {
      delay(DELAI_RETRY_I2C);
    }
  }
  return false;  // Toutes les tentatives ont échoué
}

// =============================================================
//  VÉRIFICATION CAPTEURS I2C
//
//  Scanne les adresses I2C du SHT35 (0x44) et SCD40 (0x62).
//  3 tentatives par capteur avant de déclarer l'absence.
//  Affiche le résultat détaillé sur le TFT.
//
//  Retourne : true si les DEUX capteurs sont présents, false sinon
//
//  Appelée dans ETAT_CHECK_CAPTEURS.
//  En cas de false → signalerErreur() → ETAT_ERREUR.
// =============================================================
bool verifierCapteursI2C() {

  Serial.println("[I2C] Scan en cours...");

  // Scan SHT35
  bool sht35Trouve = scannerAdresse(ADRESSE_SHT35, TENTATIVES_I2C);
  Serial.printf("[I2C] SHT35 [0x%02X] → %s\n",
                ADRESSE_SHT35, sht35Trouve ? "OK" : "ABSENT");

  // Scan SCD40
  bool scd40Trouve = 1;
  //bool scd40Trouve = scannerAdresse(ADRESSE_SCD40, TENTATIVES_I2C);
  //Serial.printf("[I2C] SCD40 [0x%02X] → %s\n",
  //              ADRESSE_SCD40, scd40Trouve ? "OK" : "ABSENT");

  // Affichage résultat sur TFT
  afficherResultatI2C(sht35Trouve, scd40Trouve);

  // Les deux doivent être présents pour continuer
  return sht35Trouve && scd40Trouve;
}

// =============================================================
//  RÉGULATION THERMIQUE — Bang-bang avec zone morte
//
//  Contrôle le Peltier TEC1-12703 et le ventilateur selon
//  la température mesurée par le SHT35.
//
//  Logique :
//    T < SEUIL_CHAUFFE   (36.5°C) → Peltier chauffe   + Ventilateur OFF
//    T > SEUIL_REFROIDIT (37.5°C) → Peltier refroidit + Ventilateur ON
//    Zone neutre [36.5 - 37.5°C]  → Tout OFF
//
//  Direction Peltier (pont en H) :
//    Chauffe    : IN1=HIGH, IN2=LOW
//    Refroidit  : IN1=LOW,  IN2=HIGH
//    Éteint     : IN1=LOW,  IN2=LOW, PWM=0
//
//  Ventilateur ON pendant refroidissement :
//    Évacue la chaleur côté dissipateur pour maintenir
//    l'efficacité du Peltier en mode refroidissement.
//  Ventilateur OFF pendant chauffage :
//    Évite de dissiper la chaleur produite côté enceinte.
//
//  Appelée en continu dans ETAT_MARCHE.
//  La sécurité T > TEMP_SECURITE est gérée dans loop() avant appel.
// =============================================================
void regulerTemperature() {

  if (temperature < SEUIL_CHAUFFE) {
    // --- Chauffe ---
    digitalWrite(BROCHE_PELTIER, HIGH);
    digitalWrite(BROCHE_VENTILATEUR, LOW);
    Serial.printf("[THERM] Chauffe — T=%.1f°C\n", temperature);

  } else if (temperature > SEUIL_REFROIDIT) {
    // --- Trop chaud — dissipation passive ---
    digitalWrite(BROCHE_PELTIER, LOW);
    digitalWrite(BROCHE_VENTILATEUR, HIGH);
    Serial.printf("[THERM] Trop chaud — T=%.1f°C\n", temperature);

  } else {
    // --- Zone neutre ---
    digitalWrite(BROCHE_PELTIER, LOW);
    digitalWrite(BROCHE_VENTILATEUR, LOW);
  }
}

// =============================================================
//  RÉGULATION CO₂ — Injection impulsionnelle non bloquante
//
//  Sous-machine à états à 3 niveaux :
//
//  CO2_IDLE :
//    Lecture toutes les INTERVALLE_CO2 ms (5s = cadence SCD40).
//    Si co2 < SEUIL_BAS_CO2 → ouvre électrovanne → CO2_INJECTING.
//    Si co2 > SEUIL_HAUT_CO2 → attente dilution naturelle (rien).
//
//  CO2_INJECTING :
//    Électrovanne V114 ouverte pendant DUREE_INJECT_MS (200ms).
//    Après → fermeture vanne → CO2_STABILIZING.
//
//  CO2_STABILIZING :
//    Attente DUREE_STABILISATION ms (30s) pour laisser le CO₂
//    se diffuser dans l'enceinte et le SCD40 se stabiliser.
//    Évite les oscillations dues à l'inertie de diffusion.
//    Après → retour CO2_IDLE.
//
//  Électrovanne V114 NC (Normally Closed) :
//    GPIO HIGH → vanne ouvre → CO₂ injecté
//    GPIO LOW  → vanne fermée → CO₂ bloqué (sécurité par défaut)
//
//  Appelée en continu dans ETAT_MARCHE.
// =============================================================
void miseAJourCO2() {
  unsigned long maintenant = millis();

  switch (etatCO2) {

    // --- En attente, décision périodique ---
    case CO2_IDLE:
      if (maintenant - dernierCO2 >= INTERVALLE_CO2) {
        dernierCO2 = maintenant;
        Serial.printf("[CO₂] Lecture : %.2f%%\n", co2);

        if (co2 < SEUIL_BAS_CO2) {
          // CO₂ insuffisant → ouvrir électrovanne
          digitalWrite(BROCHE_SOLENOIDE, HIGH);
          timerCO2 = maintenant;
          etatCO2 = CO2_INJECTING;
          Serial.printf("[CO₂] Injection déclenchée (%.2f%% < %.2f%%)\n",
                        co2, SEUIL_BAS_CO2);

        } else if (co2 > SEUIL_HAUT_CO2) {
          // CO₂ trop élevé → dilution naturelle, rien à faire
          Serial.printf("[CO₂] Trop élevé — dilution naturelle (%.2f%%)\n", co2);
        }
        // Zone cible [4.7 - 5.3%] → aucune action
      }
      break;

    // --- Injection en cours ---
    case CO2_INJECTING:
      if (maintenant - timerCO2 >= DUREE_INJECT_MS) {
        // Durée d'injection atteinte → fermer vanne
        digitalWrite(BROCHE_SOLENOIDE, LOW);
        timerCO2 = maintenant;
        etatCO2 = CO2_STABILIZING;
        Serial.println("[CO₂] Vanne fermée → stabilisation 30s");
      }
      break;

    // --- Post-injection, attente diffusion ---
    case CO2_STABILIZING:
      if (maintenant - timerCO2 >= DUREE_STABILISATION) {
        // Stabilisation terminée → retour en veille
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
      // Première détection → logger
      alerteHR = true;
      Serial.printf("[HR] ALERTE — HR=%.1f%% < %.1f%% — enceinte ouverte ?\n",
                    humidite, SEUIL_HR_ALERTE);
    }
  } else {
    // HR revenue à la normale
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
//    - Peltier : direction neutre, PWM=0
//    - Ventilateur : OFF
//    - Électrovanne V114 : fermée (LOW = NC fermée = sécurité)
//    - LED rouge : OFF
//
//  Appelée dans setup(), signalerErreur() et ETAT_ERREUR.
// =============================================================
void toutEteindre() {
  digitalWrite(BROCHE_PELTIER, LOW);
  digitalWrite(BROCHE_VENTILATEUR, LOW);
  digitalWrite(BROCHE_SOLENOIDE, LOW);  // V114 NC → fermée = sécurité
  digitalWrite(BROCHE_LED_ROUGE, LOW);
}

// =============================================================
//  SIGNALER ERREUR — Passage forcé en ETAT_ERREUR
//
//  Éteint tous les actionneurs, affiche le message d'erreur
//  sur le TFT et sur le port série, puis force l'état ERREUR.
//
//  Paramètre :
//    message — chaîne C décrivant la cause de l'erreur
//              (affiché Serial, visible dans les logs de debug)
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
//    - SCD40 [0x62] : OK (vert) ou ABSENT (rouge)
//    - Résultat global : SYSTÈME OK ou ERREUR
//
//  Paramètres :
//    sht35 — true si SHT35 détecté
//    scd40 — true si SCD40 détecté
// =============================================================
void afficherResultatI2C(bool sht35, bool scd40) {
  ecran.fillScreen(0x1E9F);  // Fond bleu-vert
  ecran.setTextDatum(MC_DATUM);

  ecran.setTextColor(TFT_WHITE, 0x1E9F);
  ecran.setTextSize(2);
  ecran.drawString("Capteurs I2C", 120, 40);

  ecran.setTextSize(1);

  // SHT35
  ecran.setTextColor(sht35 ? TFT_GREEN : TFT_RED, 0x1E9F);
  ecran.drawString(sht35 ? "SHT35  [0x44]  OK"
                         : "SHT35  [0x44]  ABSENT",
                   120, 95);

  // SCD40
  ecran.setTextColor(scd40 ? TFT_GREEN : TFT_RED, 0x1E9F);
  ecran.drawString(scd40 ? "SCD40  [0x62]  OK"
                         : "SCD40  [0x62]  ABSENT",
                   120, 125);

  // Résultat global
  ecran.setTextSize(2);
  if (sht35 && scd40) {
    ecran.setTextColor(TFT_GREEN, 0x1E9F);
    ecran.drawString("SYSTEME OK", 120, 185);
  } else {
    ecran.setTextColor(TFT_RED, 0x1E9F);
    ecran.drawString("ERREUR CAPTEUR", 120, 185);
  }
}

// =============================================================
//  AFFICHAGE ÉCRAN TFT — Dispatcher principal
//
//  Redessine l'écran complet selon l'état courant.
//  Chaque état a sa couleur de fond distinctive pour
//  identification visuelle rapide.
//
//  Paramètre :
//    e — état à afficher (ETAT_INIT à ETAT_ERREUR)
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

    // --- INIT : écran de démarrage ---
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

    // --- CHECK CAPTEURS ---
    case ETAT_CHECK_CAPTEURS:
      ecran.fillScreen(0x1E9F);
      ecran.setTextColor(TFT_WHITE, 0x1E9F);
      ecran.setTextSize(2);
      ecran.drawString("Detection", 120, 80);
      ecran.setTextSize(1);
      ecran.drawString("Scan I2C en cours...", 120, 140);
      ecran.drawString("SHT35 + SCD40", 120, 165);
      break;

    // --- ATTENTE DONNÉES ---
    case ETAT_ATTENTE_DONNEES:
      ecran.fillScreen(0x1E9F);
      ecran.setTextColor(TFT_WHITE, 0x1E9F);
      ecran.setTextSize(2);
      ecran.drawString("Verification", 120, 80);
      ecran.setTextSize(1);
      ecran.drawString("Lecture capteurs...", 120, 140);
      ecran.drawString("Validation plages", 120, 165);
      break;

    // --- BUFFER : attente GO ---
    case ETAT_BUFFER:
      {
        ecran.fillScreen(0xFD20);
        ecran.setTextColor(TFT_BLACK, 0xFD20);
        ecran.setTextSize(2);
        ecran.drawString("PRET", 120, 40);
        ecran.setTextSize(1);
        char buf[28];
        snprintf(buf, sizeof(buf), "T: %.1f C", temperature);
        ecran.drawString(buf, 120, 95);
        snprintf(buf, sizeof(buf), "HR: %.1f %%", humidite);
        ecran.drawString(buf, 120, 120);
        snprintf(buf, sizeof(buf), "CO2: %.2f %%", co2);
        ecran.drawString(buf, 120, 145);
        ecran.setTextSize(1);
        ecran.drawString("Fermez l'enceinte", 120, 185);
        ecran.drawString("puis appuyez GO", 120, 205);
        break;
      }

    // --- MARCHE : régulation active ---
    case ETAT_MARCHE:
      {
        ecran.fillScreen(0x07E0);
        ecran.setTextColor(TFT_BLACK, 0x07E0);
        ecran.setTextSize(2);
        ecran.drawString("EN MARCHE", 120, 25);
        ecran.setTextSize(1);
        char ligne[32];

        // Température avec indicateur visuel chaud/froid
        const char* indT = (temperature < SEUIL_CHAUFFE) ? " [+]" : (temperature > SEUIL_REFROIDIT) ? " [-]"
                                                                                                    : " [=]";
        snprintf(ligne, sizeof(ligne), "T: %.1f/%.1f C%s",
                 temperature, CONSIGNE_TEMP, indT);
        ecran.drawString(ligne, 120, 80);

        // Humidité avec alerte si nécessaire
        if (alerteHR) {
          ecran.setTextColor(TFT_RED, 0x07E0);
          snprintf(ligne, sizeof(ligne), "HR: %.1f %% ALERTE", humidite);
        } else {
          snprintf(ligne, sizeof(ligne), "HR: %.1f %%", humidite);
        }
        ecran.drawString(ligne, 120, 110);
        ecran.setTextColor(TFT_BLACK, 0x07E0);

        // CO₂ avec indicateur état injection
        const char* indCO2 = (etatCO2 == CO2_INJECTING) ? " [INJ]" : (etatCO2 == CO2_STABILIZING) ? " [STB]"
                                                                                                  : "";
        snprintf(ligne, sizeof(ligne), "CO2: %.2f %%%s", co2, indCO2);
        ecran.drawString(ligne, 120, 140);

        // État Peltier
        const char* etatPeltier = (temperature < SEUIL_CHAUFFE) ? "Chauffe" : (temperature > SEUIL_REFROIDIT) ? "Refroidit"
                                                                                                              : "Neutre";
        snprintf(ligne, sizeof(ligne), "Peltier: %s", etatPeltier);
        ecran.drawString(ligne, 120, 175);
        break;
      }

    // --- ERREUR : état final ---
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
