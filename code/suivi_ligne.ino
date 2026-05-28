#include <Wire.h>   // bus I2C : moteurs + capteur suiveur de ligne

// ============================================
// ADRESSES I2C
// ============================================
#define MOTEUR_G            0x66
#define MOTEUR_D            0x68
#define ADDR_LF             0x20   // capteur de ligne (Line Follower)

// ============================================
// COMMANDES MOTEUR
// ============================================
#define ARRET    0x00
#define AVANT    0x01
#define ARRIERE  0x02
#define FREIN    0x03

// ============================================
// REGISTRES Me RGB LINE FOLLOWER
// ============================================
#define REG_MODE      0x01   // mode (0 = normal)
#define REG_ANALOG_1  0x01   // niveau brut capteur 1 (~0x90 blanc, ~0x09 noir)
#define REG_ANALOG_2  0x02
#define REG_ANALOG_3  0x03
#define REG_ANALOG_4  0x04
#define REG_QUALITY   0x05   // 0xD0 blanc / 0x00 noir
#define REG_LINE_OK   0x06   // 0xFE si ligne détectée / 0x00 sinon
#define REG_DIGITAL   0x07   // ← LE registre clé : 4 bits = état des 4 capteurs

// Dans R07 : bit=1 -> blanc | bit=0 -> noir (ligne)
// On INVERSE pour avoir : bit=1 -> sur la ligne (plus intuitif)

// ============================================
// PARAMÈTRES DE PILOTAGE
// ============================================
#define VITESSE_BASE  40   // vitesse nominale en ligne droite
#define CORRECTION    15   // écart de vitesse appliqué pour corriger la trajectoire

uint8_t dernierEtat = 0b0110;   // mémoire du dernier état valide (centré par défaut)

// ============================================
void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(100);

  // Sécurité : moteurs à l'arrêt au démarrage
  piloterMoteur(MOTEUR_G, ARRET, 0);
  piloterMoteur(MOTEUR_D, ARRET, 0);

  // Pas besoin d'init du capteur, il marche en mode normal par défaut
  
  Serial.println(F("=== Suiveur de ligne (robot retourne) ==="));
  Serial.println(F("Place le robot sur la ligne..."));

  // Attente du déclenchement : on ne part que lorsqu'un capteur voit la ligne
  while (true) {
    uint8_t etat = lireCapteurs();
    Serial.print(F("Etat : "));
    afficheBin(etat);
    Serial.println();
    
    if (etat != 0b0000) {   // au moins un capteur sur la ligne
      Serial.println(F("Ligne detectee ! Demarrage..."));
      break;
    }
    delay(100);
  }
  
  delay(500);  // petit délai avant de partir
}

// ============================================
// Boucle principale : on lit l'état des 4 capteurs et on choisit
// la commande moteur correspondante (machine à états simple).
// ============================================
void loop() {
  uint8_t etat = lireCapteurs();
  
  // Si on perd la ligne, on garde le dernier état mémorisé
  // (utile pour ne pas s'arrêter brutalement dans un virage serré)
  if (etat != 0b0000) {
    dernierEtat = etat;
  }

  Serial.print(F("R07="));
  afficheBin(etat);
  Serial.print(F("  -> "));

  switch (etat) {

    // ----- Centré sur la ligne -----
    case 0b0110:
      Serial.println(F("CENTRE"));
      avancer(VITESSE_BASE, VITESSE_BASE);
      break;

    // ----- Très large (intersection ou ligne épaisse) -----
    case 0b1111:
    case 0b0111:
    case 0b1110:
      Serial.println(F("LARGE/INTERSECTION"));
      avancer(VITESSE_BASE, VITESSE_BASE);
      break;

    // ----- Léger décalage vers la gauche du robot -----
    // (la ligne est à droite par rapport au robot)
    case 0b0100:
      Serial.println(F("Leger droit"));
      avancer(VITESSE_BASE + CORRECTION, VITESSE_BASE - CORRECTION);
      break;

    case 0b1100:
      Serial.println(F("Virage droit"));
      avancer(VITESSE_BASE, 0);
      break;

    case 0b1000:
      Serial.println(F("Virage droit serre"));
      avancer(VITESSE_BASE, 0);
      break;

    // ----- Léger décalage vers la droite du robot -----
    case 0b0010:
      Serial.println(F("Leger gauche"));
      avancer(VITESSE_BASE - CORRECTION, VITESSE_BASE + CORRECTION);
      break;

    case 0b0011:
      Serial.println(F("Virage gauche"));
      avancer(0, VITESSE_BASE);
      break;

    case 0b0001:
      Serial.println(F("Virage gauche serre"));
      avancer(0, VITESSE_BASE);
      break;

    // ----- Ligne perdue : tentative de rattrapage -----
    case 0b0000:
      Serial.print(F("PERDU (rattrap dernier="));
      afficheBin(dernierEtat);
      Serial.println(F(")"));
      rattrapage(dernierEtat);
      break;

    // ----- Cas non prévu : on s'arrête par sécurité -----
    default:
      Serial.println(F("INCONNU - stop"));
      piloterMoteur(MOTEUR_G, ARRET, 0);
      piloterMoteur(MOTEUR_D, ARRET, 0);
      break;
  }

  delay(20);   // période de la boucle d'asservissement
}

// ============================================
// LECTURE CAPTEURS — R07 inversé pour avoir
// bit=1 = sur la ligne
// ============================================
uint8_t lireCapteurs() {
  Wire.beginTransmission(ADDR_LF);
  Wire.write(REG_DIGITAL);
  if (Wire.endTransmission(false) != 0) return 0;   // échec com -> état neutre
  
  Wire.requestFrom(ADDR_LF, (uint8_t)1);
  if (!Wire.available()) return 0;
  
  uint8_t brut = Wire.read();
  return (~brut) & 0x0F;   // inversion + masque sur les 4 bits utiles
}

// ============================================
// RATTRAPAGE quand ligne perdue
// On repart dans la direction du dernier côté où la ligne a été vue.
// ============================================
void rattrapage(uint8_t dernier) {
  if (dernier & 0b1100) {
    // ligne était à gauche → tourner à gauche (sens inversé)
    piloterMoteur(MOTEUR_G, AVANT, VITESSE_BASE / 2);
    piloterMoteur(MOTEUR_D, AVANT, VITESSE_BASE / 2);
  } else if (dernier & 0b0011) {
    // ligne était à droite → tourner à droite (sens inversé)
    piloterMoteur(MOTEUR_G, ARRIERE, VITESSE_BASE / 2);
    piloterMoteur(MOTEUR_D, ARRIERE, VITESSE_BASE / 2);
  } else {
    piloterMoteur(MOTEUR_G, ARRET, 0);
    piloterMoteur(MOTEUR_D, ARRET, 0);
  }
}

// ============================================
// AVANCER — sens des moteurs INVERSE
// (robot retourne : grosses roues a l'avant)
// Moteur G : ARRIERE pour avancer
// Moteur D : AVANT    pour avancer
// ============================================
void avancer(int vitesseG, int vitesseD) {
  vitesseG = constrain(vitesseG, 0, 63);
  vitesseD = constrain(vitesseD, 0, 63);
  piloterMoteur(MOTEUR_G, vitesseG > 0 ? ARRIERE : ARRET, vitesseG);
  piloterMoteur(MOTEUR_D, vitesseD > 0 ? AVANT   : ARRET, vitesseD);
}

// ============================================
// Trame I2C de commande moteur : vitesse (6 bits) + direction (2 bits)
// ============================================
void piloterMoteur(byte adresse, byte direction, byte vitesse) {
  if (vitesse > 63) vitesse = 63;
  byte commande = (vitesse << 2) | direction;
  Wire.beginTransmission(adresse);
  Wire.write(0x00);
  Wire.write(commande);
  Wire.endTransmission();
}

// ============================================
// Affiche un octet en binaire sur 4 bits (debug capteurs)
// ============================================
void afficheBin(uint8_t v) {
  for (int i = 3; i >= 0; i--) Serial.print((v >> i) & 1);
}
