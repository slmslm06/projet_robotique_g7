#include <Wire.h>    // bus I2C (moteurs + capteur de ligne)
#include <Servo.h>   // orientation de l'ultrason

// --- MATÉRIEL ET ADRESSES ---
#define MOTEUR_G 0x66
#define MOTEUR_D 0x68
#define ADDR_LF  0x20      // capteur suiveur de ligne
#define REG_DIGITAL 0x07   // registre des 4 sorties tout-ou-rien du capteur

#define ARRET    0x00
#define AVANT    0x01
#define ARRIERE  0x02
#define FREIN    0x03

#define ULTRASON_PIN 4
#define SERVO_PIN 6

// --- PARAMÈTRES SUIVI DE LIGNE (Ton code d'origine) ---
#define VITESSE_BASE 40
#define CORRECTION   15
#define TRIM_G      -3   // correction d'équilibrage moteur gauche
#define TRIM_D       0   // correction d'équilibrage moteur droit

// --- PARAMÈTRES SERVO ---
#define SERVO_CENTRE 90
#define SERVO_DROITE 0    // ultrason pointé vers la droite (suivi de mur)

Servo monServo;

// Machine à états : deux phases du parcours.
enum Etat {
  SECTION_1,   // suivi de ligne classique
  SECTION_2    // traversée du tunnel (suivi de mur à l'ultrason)
};

Etat etatActuel = SECTION_1;

// Gestion de la perte de ligne (détection de l'entrée du tunnel)
unsigned long tempsLignePerdue = 0;
bool lignePerdue = false;
uint8_t dernierEtatCapteurs = 0b0110;   // dernier état valide (centré)

// ================= MOTEURS (Ton code d'origine) =================
// Trame I2C : vitesse (6 bits) + direction (2 bits).
void piloterMoteur(byte adresse, byte direction, byte vitesse) {
  if (vitesse > 63) vitesse = 63;
  byte commande = (vitesse << 2) | direction;
  Wire.beginTransmission(adresse);
  Wire.write(0x00);
  Wire.write(commande);
  Wire.endTransmission();
}

// Marche avant avec correction de trim et bornage de la vitesse.
void avancer(int vitesseG, int vitesseD) {
  vitesseG = vitesseG + TRIM_G;
  vitesseD = vitesseD + TRIM_D;
  vitesseG = constrain(vitesseG, 0, 63);
  vitesseD = constrain(vitesseD, 0, 63);
  // Sur ton robot, le moteur gauche doit être en ARRIERE pour avancer
  piloterMoteur(MOTEUR_G, vitesseG > 0 ? ARRIERE : ARRET, vitesseG);
  piloterMoteur(MOTEUR_D, vitesseD > 0 ? AVANT   : ARRET, vitesseD);
}

void arreter() {
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
}

// ================= CAPTEUR LIGNE (Ton code d'origine) =================
// Retourne l'état des 4 capteurs (bit=1 -> sur la ligne).
uint8_t lireCapteurs() {
  Wire.beginTransmission(ADDR_LF);
  Wire.write(REG_DIGITAL);
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom(ADDR_LF, (uint8_t)1);
  if (!Wire.available()) return 0;
  uint8_t brut = Wire.read();
  // Cette ligne est cruciale : elle inverse les bits pour correspondre à ta logique
  return (~brut) & 0x0F;
}

// ================= SUIVI DE LIGNE (Ton code d'origine) =================
// Choisit la commande moteur selon la position de la ligne sous les capteurs.
void piloterSelonEtat(uint8_t e) {
  switch (e) {
    case 0b0110:   // centré -> tout droit
      avancer(VITESSE_BASE, VITESSE_BASE);
      break;
    case 0b0100:   // léger décalage -> petite correction
      avancer(VITESSE_BASE + CORRECTION, VITESSE_BASE - CORRECTION);
      break;
    case 0b1100:
    case 0b1000:   // ligne franchement d'un côté -> virage marqué
      avancer(VITESSE_BASE, 0);
      break;
    case 0b0010:   // décalage de l'autre côté
      avancer(VITESSE_BASE - CORRECTION, VITESSE_BASE + CORRECTION);
      break;
    case 0b0011:
    case 0b0001:
      avancer(0, VITESSE_BASE);
      break;
    case 0b0000:   // ligne perdue -> on rejoue le dernier état connu
      piloterSelonEtat(dernierEtatCapteurs);
      break;
  }
}

// ================= ULTRASON =================
// Mesure mono-broche, retourne la distance en cm (999 si pas d'écho).
long lireDistance() {
  pinMode(ULTRASON_PIN, OUTPUT);
  digitalWrite(ULTRASON_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASON_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASON_PIN, LOW);

  pinMode(ULTRASON_PIN, INPUT);
  long duree = pulseIn(ULTRASON_PIN, HIGH, 25000);

  if (duree == 0) return 999;
  return duree / 58;
}

// ================= NOUVEAU TUNNEL =================
// Dans le tunnel : on suit le mur de droite en gardant une distance
// de consigne (~26 cm) grâce à l'ultrason orienté à droite.
void suivreTunnelMurDroit() {
  monServo.write(SERVO_DROITE);
  delay(50); // Laisse le temps au servo de rester en place (ou d'y aller)

  long distD = lireDistance();

  Serial.print("Tunnel distD = ");
  Serial.println(distD);

  // Asservissement simple type tout-ou-rien autour de la consigne.
  if (distD < 24) {
    // trop proche du mur droit -> braque vers la gauche
    avancer(8, 35);
  }
  else if (distD > 28) {
    // trop loin du mur droit -> braque vers la droite
    avancer(35, 8);
  }
  else {
    // bonne zone : environ 26 cm -> avance droit
    avancer(23, 23);
  }
}

// ================= SETUP =================
void setup() {
  Wire.begin();
  Serial.begin(9600);

  monServo.attach(SERVO_PIN);
  monServo.write(SERVO_CENTRE);

  arreter();

  // Démarrage : on attend que le robot soit posé sur la ligne
  Serial.println("Place le robot sur la ligne de depart...");
  while (lireCapteurs() == 0b0000) {
    delay(100);
  }
  Serial.println("GO !");
  delay(500);
}

// ================= LOOP =================
// Boucle principale pilotée par la machine à états (ligne puis tunnel).
void loop() {
  uint8_t e = lireCapteurs();

  switch (etatActuel) {

    // ================= SECTION 1 : LIGNE =================
    case SECTION_1:
      if (e != 0b0000) {
        // ligne visible : on mémorise et on remet à zéro le compteur de perte
        dernierEtatCapteurs = e;
        lignePerdue = false;
        tempsLignePerdue = 0;
      } else {
        // ligne perdue : on démarre le chrono à la première perte
        if (!lignePerdue) {
          lignePerdue = true;
          tempsLignePerdue = millis();
        }
      }

      // Ligne absente depuis plus de 500 ms -> on considère être au tunnel
      if (lignePerdue && (millis() - tempsLignePerdue > 500)) {
        arreter();
        delay(100);

        monServo.write(SERVO_DROITE);   // oriente l'ultrason vers le mur droit
        delay(200);

        etatActuel = SECTION_2;
        Serial.println("PASSAGE SECTION 2 : TUNNEL");
      } else {
        piloterSelonEtat(e);   // sinon : on continue le suivi de ligne
      }
      break;

    // ================= SECTION 2 : TUNNEL =================
    case SECTION_2:
      suivreTunnelMurDroit();

      // Sortie du tunnel détectée quand un capteur retrouve la ligne
      if (lireCapteurs() != 0b0000) {
        arreter();
        delay(150);

        monServo.write(SERVO_CENTRE);   // ultrason recentré
        delay(200);

        lignePerdue = false;
        tempsLignePerdue = 0;

        etatActuel = SECTION_1;
        Serial.println("SORTIE TUNNEL -> RETOUR LIGNE");
      }
      break;
  }

  delay(30);   // période de la boucle de contrôle
}
