// =============================================================
//  Configuration de la bibliothèque TFT_eSPI pour l'écran du PB-0326
//
//  ⚠️ Fichier RECONSTITUÉ d'après les commentaires du firmware
//     (versions 3.3 et 4.0.1). Le fichier d'origine n'a pas été conservé :
//     vérifier l'affichage au premier essai.
//
//  Installation : remplacer le fichier User_Setup.h du dossier de la
//  bibliothèque TFT_eSPI (Documents/Arduino/libraries/TFT_eSPI/)
//  par celui-ci, puis recompiler.
// =============================================================

#define USER_SETUP_INFO "PB-0326 — ST7789 240x280"

// ── Pilote et taille de l'écran ──
#define ST7789_DRIVER
#define TFT_WIDTH  240
#define TFT_HEIGHT 280

#define TFT_RGB_ORDER TFT_RGB
#define TFT_INVERSION_ON

// ── Broches ESP32 (bus VSPI) ──
#define TFT_MOSI 23
#define TFT_SCLK 18
#define TFT_CS   15
#define TFT_DC    2
#define TFT_RST  33
#define TFT_MISO -1   // non utilisé par l'écran

// ── Polices ──
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF
#define SMOOTH_FONT

// ── Fréquence SPI ──
#define SPI_FREQUENCY  40000000
