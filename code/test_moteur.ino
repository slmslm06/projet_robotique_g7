#include <Wire.h>   // Bus I2C : on dialogue avec les drivers moteurs par ce bus

// --- Adresses I2C des deux drivers moteurs ---
#define MOTEUR_G  0x66
#define MOTEUR_D  0x68

// --- Codes de commande envoyés au driver ---
#define ARRET   0x00   // roue libre (pas d'alimentation)
#define AVANT   0x01
#define ARRIERE 0x02
#define FREIN   0x03   // court-circuit moteur = arrêt rapide

// Envoie une consigne de direction + vitesse à un driver moteur via I2C.
// La trame est codée sur un octet : 6 bits de vitesse + 2 bits de direction.
void piloterMoteur(byte adresse, byte direction, byte vitesse) {
  if (vitesse > 63) vitesse = 63;                 // vitesse sur 6 bits -> max 63
  byte commande = (vitesse << 2) | direction;     // [vitesse(6) | direction(2)]
  Wire.beginTransmission(adresse);
  Wire.write(0x00);                               // registre cible du driver
  Wire.write(commande);
  Wire.endTransmission();
}

void setup() {
  Wire.begin();           // initialisation I2C (maître)
  Serial.begin(9600);     // moniteur série pour suivre le test
  Serial.println("GO");
}

// Boucle de test : on alterne marche avant / arrêt / marche arrière.
// Rappel : le robot étant "retourné", les deux moteurs tournent en sens
// opposés (un AVANT, un ARRIERE) pour avancer en ligne droite.
void loop() {
  Serial.println("Les deux AVANT");
  piloterMoteur(MOTEUR_G, AVANT, 40);
  piloterMoteur(MOTEUR_D, ARRIERE, 40);
  delay(2000);

  Serial.println("Stop");
  piloterMoteur(MOTEUR_G, ARRET, 0);
  piloterMoteur(MOTEUR_D, ARRET, 0);
  delay(1000);

  Serial.println("Les deux ARRIERE");
  piloterMoteur(MOTEUR_G, ARRIERE, 40);
  piloterMoteur(MOTEUR_D, AVANT, 40);
  delay(2000);

  Serial.println("Stop");
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
  delay(2000);
}
