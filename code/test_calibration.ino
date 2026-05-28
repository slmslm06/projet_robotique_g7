// ============================================================
//  PROGRAMME DE TEST ET CALIBRATION - Manœuvres robot
//
//  Ouvrir le moniteur série à 9600 bauds
//  Taper une lettre + Entrée pour lancer l'action
// ============================================================

#include <Wire.h>
#include <Servo.h>

// ============================================================
// BROCHES ET ADRESSES — identiques au programme principal
// ============================================================
#define PIN_SERVO     6
#define PIN_ULTRASON  4
#define MOTEUR_G      0x66
#define MOTEUR_D      0x68
#define ARRET         0x00
#define AVANT         0x01
#define ARRIERE       0x02
#define FREIN         0x03

#define VITESSE_LENTE  30   // vitesse réduite pour les essais (plus précis)

// ============================================================
// ⚠️  METTRE ICI TES VALEURS EN COURS DE CALIBRATION  ⚠️
// Ces deux constantes traduisent une consigne (cm ou deg) en durée (ms).
// Ajuste ces valeurs et recharge le programme à chaque essai
// ============================================================
#define DUREE_TOURNE_90   1000   // ms pour une rotation de 90° - à ajuster
#define DUREE_PAR_CM       60    // ms par cm parcouru - à ajuster

Servo servoUs;

// ============================================================
void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(200);

  servoUs.attach(PIN_SERVO);
  servoUs.write(90);   // servo centré
  delay(500);

  // Moteurs à l'arrêt au démarrage
  piloterMoteur(MOTEUR_G, ARRET, 0);
  piloterMoteur(MOTEUR_D, ARRET, 0);

  afficherMenu();
}

// ============================================================
// Attente d'une commande clavier puis exécution de l'action associée.
// ============================================================
void loop() {
  if (!Serial.available()) return;

  char c = Serial.read();
  while (Serial.available()) Serial.read(); // vider le buffer

  Serial.println();

  switch (c) {

    // --- AVANCEMENT ---
    case 'a':   // distance théorique calculée à partir de DUREE_PAR_CM
      Serial.println(F("Avancer 30 cm..."));
      Serial.println(F("Mesure la distance avec une regle."));
      Serial.print  (F("Duree theorique : "));
      Serial.print  (30 * DUREE_PAR_CM);
      Serial.println(F(" ms"));
      delay(2000); // 2s pour te préparer
      avancerDroit(VITESSE_LENTE, 30 * DUREE_PAR_CM);
      Serial.println(F("Termine. Distance mesuree = ? cm"));
      Serial.println(F("Nouvelle valeur DUREE_PAR_CM = 30 * 1000 / distance_cm"));
      break;

    case 'A':   // durée brute fixe, sert à déterminer DUREE_PAR_CM
      Serial.println(F("Avancer 3 secondes (valeur brute)..."));
      delay(2000);
      avancerDroit(VITESSE_LENTE, 3000);
      Serial.println(F("Termine. Mesure la distance parcourue."));
      Serial.println(F("DUREE_PAR_CM = 3000 / distance_mesure"));
      break;

    // --- ROTATIONS ---
    case 'd':
      Serial.println(F("Tourner DROITE 90 degres (theorique)..."));
      Serial.println(F("Place un rapporteur sous le robot."));
      delay(2000);
      tournerDroite(DUREE_TOURNE_90);
      Serial.println(F("Angle mesure = ? degres"));
      Serial.print  (F("Nouvelle valeur DUREE_TOURNE_90 = "));
      Serial.print  (DUREE_TOURNE_90);
      Serial.println(F(" * 90 / angle_mesure"));
      break;

    case 'g':
      Serial.println(F("Tourner GAUCHE 90 degres (theorique)..."));
      delay(2000);
      tournerGauche(DUREE_TOURNE_90);
      Serial.println(F("Angle mesure = ? degres"));
      break;

    case 'D':   // rotation brute 1 s pour calibrer DUREE_TOURNE_90
      Serial.println(F("Tourner DROITE 1 seconde (brut)..."));
      delay(2000);
      tournerDroite(1000);
      Serial.println(F("Angle mesure = ? degres"));
      Serial.println(F("DUREE_TOURNE_90 = 1000 * 90 / angle_mesure"));
      break;

    case 'G':
      Serial.println(F("Tourner GAUCHE 1 seconde (brut)..."));
      delay(2000);
      tournerGauche(1000);
      Serial.println(F("Angle mesure = ? degres"));
      break;

    // --- TEST MANOEUVRE COMPLETE ---
    case '1':   // enchaînement complet d'évitement par la gauche
      Serial.println(F("Test sequence EVITEMENT GAUCHE (O1)..."));
      Serial.println(F("Verifie que le robot fait bien un carre."));
      delay(3000);
      testEvitementGauche();
      break;

    case '2':
      Serial.println(F("Test sequence EVITEMENT DROITE (O2)..."));
      delay(3000);
      testEvitementDroite();
      break;

    // --- TEST ULTRASON ---
    case 'u':
      Serial.println(F("Mesures ultrason en continu (15s)..."));
      testUltrason();
      break;

    // --- TEST LIGNE DROITE ---
    case 'l':
      Serial.println(F("Ligne droite 5 secondes..."));
      Serial.println(F("Verifier que le robot ne derive pas."));
      delay(2000);
      avancerDroit(VITESSE_LENTE, 5000);
      break;

    // --- RECUL ---
    case 'r':
      Serial.println(F("Reculer 10 cm..."));
      delay(2000);
      reculer(VITESSE_LENTE, 10 * DUREE_PAR_CM);
      break;

    // --- STOP ---
    case 's':
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      Serial.println(F("STOP"));
      break;

    default:
      Serial.println(F("Commande inconnue."));
      break;
  }

  delay(500);
  afficherMenu();
}

// ============================================================
// MANOEUVRE EVITEMENT GAUCHE COMPLETE (version test)
// Trajectoire en rectangle : on contourne l'obstacle par la gauche
// puis on revient sur l'axe initial, décalé de 70 cm.
// ============================================================
void testEvitementGauche() {
  reculer(VITESSE_LENTE,   5 * DUREE_PAR_CM);
  tournerGauche(DUREE_TOURNE_90);
  avancerDroit(VITESSE_LENTE, 35 * DUREE_PAR_CM);
  tournerDroite(DUREE_TOURNE_90);
  avancerDroit(VITESSE_LENTE, 70 * DUREE_PAR_CM);
  tournerDroite(DUREE_TOURNE_90);
  avancerDroit(VITESSE_LENTE, 35 * DUREE_PAR_CM);
  tournerGauche(DUREE_TOURNE_90);
  Serial.println(F("Fin evitement gauche. Le robot doit etre"));
  Serial.println(F("revenu sur sa trajectoire initiale, decale de 70cm."));
}

// Même manœuvre, symétrique (contournement par la droite).
void testEvitementDroite() {
  reculer(VITESSE_LENTE,   5 * DUREE_PAR_CM);
  tournerDroite(DUREE_TOURNE_90);
  avancerDroit(VITESSE_LENTE, 35 * DUREE_PAR_CM);
  tournerGauche(DUREE_TOURNE_90);
  avancerDroit(VITESSE_LENTE, 70 * DUREE_PAR_CM);
  tournerGauche(DUREE_TOURNE_90);
  avancerDroit(VITESSE_LENTE, 35 * DUREE_PAR_CM);
  tournerDroite(DUREE_TOURNE_90);
  Serial.println(F("Fin evitement droite."));
}

// ============================================================
// Mesures ultrason en continu pendant 15 s (vérification du capteur).
// ============================================================
void testUltrason() {
  unsigned long fin = millis() + 15000;
  servoUs.write(90);
  delay(300);
  while (millis() < fin) {
    Serial.print(F("Distance : "));
    Serial.print(mesureUltrason());
    Serial.println(F(" cm"));
    delay(500);
  }
}

// ============================================================
void afficherMenu() {
  Serial.println(F("\n=== MENU CALIBRATION ==="));
  Serial.println(F("Tape une lettre et appuie sur ENTREE :"));
  Serial.println(F("  a  = Avancer 30 cm (distance theorique)"));
  Serial.println(F("  A  = Avancer 3 secondes (brut)"));
  Serial.println(F("  d  = Tourner DROITE 90 degres (theorique)"));
  Serial.println(F("  g  = Tourner GAUCHE 90 degres (theorique)"));
  Serial.println(F("  D  = Tourner DROITE 1 seconde (brut)"));
  Serial.println(F("  G  = Tourner GAUCHE 1 seconde (brut)"));
  Serial.println(F("  1  = Test evitement GAUCHE complet"));
  Serial.println(F("  2  = Test evitement DROITE complet"));
  Serial.println(F("  u  = Mesure ultrason en continu"));
  Serial.println(F("  l  = Ligne droite 5 secondes"));
  Serial.println(F("  r  = Reculer 10 cm"));
  Serial.println(F("  s  = STOP moteurs"));
  Serial.println(F("========================"));
}

// ============================================================
// FONCTIONS DE DEPLACEMENT
// Principe commun : on lance le mouvement, on attend une durée, on freine.
// ============================================================
void avancerDroit(int vitesse, unsigned long duree) {
  avancer(vitesse, vitesse);
  delay(duree);
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
  delay(50);
}

void reculer(int vitesse, unsigned long duree) {
  // Sens des moteurs inversé par rapport à avancer()
  piloterMoteur(MOTEUR_G, AVANT,   vitesse);
  piloterMoteur(MOTEUR_D, ARRIERE, vitesse);
  delay(duree);
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
  delay(50);
}

void tournerGauche(unsigned long duree) {
  // ⚠️ Si le robot tourne dans le mauvais sens,
  // echange AVANT et ARRIERE ici ET dans tournerDroite
  // Les 2 moteurs dans le même sens -> rotation sur place
  piloterMoteur(MOTEUR_G, AVANT, VITESSE_LENTE);
  piloterMoteur(MOTEUR_D, AVANT, VITESSE_LENTE);
  delay(duree);
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
  delay(50);
}

void tournerDroite(unsigned long duree) {
  piloterMoteur(MOTEUR_G, ARRIERE, VITESSE_LENTE);
  piloterMoteur(MOTEUR_D, ARRIERE, VITESSE_LENTE);
  delay(duree);
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
  delay(50);
}

// Marche avant : moteurs en sens opposés (robot retourné).
void avancer(int vitesseG, int vitesseD) {
  vitesseG = constrain(vitesseG, 0, 63);
  vitesseD = constrain(vitesseD, 0, 63);
  piloterMoteur(MOTEUR_G, vitesseG > 0 ? ARRIERE : ARRET, vitesseG);
  piloterMoteur(MOTEUR_D, vitesseD > 0 ? AVANT   : ARRET, vitesseD);
}

// Mesure ultrason mono-broche, retourne la distance en cm (9999 si timeout).
long mesureUltrason() {
  pinMode(PIN_ULTRASON, OUTPUT);
  digitalWrite(PIN_ULTRASON, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASON, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASON, LOW);
  pinMode(PIN_ULTRASON, INPUT);
  long duree = pulseIn(PIN_ULTRASON, HIGH, 30000UL);
  if (duree == 0) return 9999;
  return duree / 58;
}

// Trame I2C de commande moteur : vitesse (6 bits) + direction (2 bits).
void piloterMoteur(byte adresse, byte direction, byte vitesse) {
  if (vitesse > 63) vitesse = 63;
  byte commande = (vitesse << 2) | direction;
  Wire.beginTransmission(adresse);
  Wire.write(0x00);
  Wire.write(commande);
  Wire.endTransmission();
}
