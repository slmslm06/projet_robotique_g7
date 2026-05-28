#include <Wire.h>                 // bus I2C (capteur de couleur)
#include <Adafruit_NeoPixel.h>    // pilotage du ruban de LED adressables

// === Configuration du capteur de couleur (registres I2C) ===
#define ADDR    0x29   // adresse I2C du capteur
#define ENABLE  0x00   // registre d'activation
#define ATIME   0x01   // temps d'intégration de la mesure
#define CONTROL 0x0F   // gain
#define RDATAL  0x16   // octet bas de la composante Rouge
#define GDATAL  0x18   // octet bas de la composante Verte
#define BDATAL  0x1A   // octet bas de la composante Bleue

// === Configuration du ruban LED ===
#define PIN_LED 3
#define NB_LEDS 30

Adafruit_NeoPixel ruban(NB_LEDS, PIN_LED, NEO_GRB + NEO_KHZ800);

// Écriture d'un octet dans un registre du capteur.
// Le bit 0x80 indique au capteur une commande d'accès registre.
void ecrireRegistre(byte registre, byte valeur) {
  Wire.beginTransmission(ADDR);
  Wire.write(0x80 | registre);
  Wire.write(valeur);
  Wire.endTransmission();
}

// Lecture d'une valeur sur 16 bits (2 octets : poids faible puis poids fort).
int lireRegistre(byte registre) {
  Wire.beginTransmission(ADDR);
  Wire.write(0x80 | registre);
  Wire.endTransmission();
  Wire.requestFrom(ADDR, 2);
  return Wire.read() | (Wire.read() << 8);
}

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialisation du capteur : temps d'intégration, gain, puis activation
  ecrireRegistre(ATIME, 0xF6);
  ecrireRegistre(CONTROL, 0x01);
  ecrireRegistre(ENABLE, 0x03);
  delay(100);

  ruban.begin();
  ruban.show();   // éteint toutes les LED au démarrage
}

void loop() {
  // === Lecture des trois composantes de couleur ===
  int rouge = lireRegistre(RDATAL);
  int vert  = lireRegistre(GDATAL);
  int bleu  = lireRegistre(BDATAL);

  Serial.print("R:"); Serial.print(rouge);
  Serial.print(" V:"); Serial.print(vert);
  Serial.print(" B:"); Serial.println(bleu);

  // === Détermination de la couleur dominante ===
  // On compare simplement les trois composantes entre elles.
  int r_led, v_led, b_led;

  if (rouge > vert && rouge > bleu) {
    r_led = 255; v_led = 0; b_led = 0;
    Serial.println("Couleur détectée : ROUGE");
  } else if (vert > rouge && vert > bleu) {
    r_led = 0; v_led = 255; b_led = 0;
    Serial.println("Couleur détectée : VERT");
  } else {
    r_led = 0; v_led = 0; b_led = 255;
    Serial.println("Couleur détectée : BLEU");
  }

  // === Clignotement du ruban pendant 3 s à 2 Hz ===
  // 6 demi-périodes de 500 ms : alternance allumé / éteint.
  for (int i = 0; i < 6; i++) {
    if (i % 2 == 0) {
      for (int j = 0; j < NB_LEDS; j++) {
        ruban.setPixelColor(j, ruban.Color(r_led, v_led, b_led));
      }
      ruban.show();
    } else {
      ruban.clear();
      ruban.show();
    }
    delay(500);
  }

  ruban.clear();
  ruban.show();
  Serial.println("Séquence terminée, prêt pour la prochaine détection.");
  delay(1000);   // pause avant la prochaine mesure
}
