#include <Arduino.h>
#include "Ultrasonic.h"   // bibliothèque Grove pour le capteur ultrason
#include "rgb_lcd.h"      // bibliothèque Grove pour l'afficheur LCD RGB

// --- Objets matériels ---
Ultrasonic ultrasonic(4);   // capteur ultrason branché sur le port D4
rgb_lcd lcd;                // afficheur LCD 16x2

long distance;   // distance mesurée, en cm

void setup() {
  Serial.begin(9600);

  // Initialisation de l'écran : 16 colonnes, 2 lignes, fond bleu
  lcd.begin(16, 2);
  lcd.setRGB(0, 128, 255);
  lcd.print("Initialisation...");
  delay(2000);
  lcd.clear();
}

void loop() {
  delay(500);                      // période d'échantillonnage : une mesure / 0,5 s
  distance = ultrasonic.read();    // lecture de la distance (cm)

  // Trace sur le moniteur série (debug)
  Serial.print("Distance : ");
  Serial.print(distance);
  Serial.println(" cm");

  // Affichage sur le LCD : libellé sur la 1re ligne, valeur sur la 2e
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Distance:");
  lcd.setCursor(0, 1);
  lcd.print(distance);
  lcd.print(" cm");
}
