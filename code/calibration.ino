#include <Wire.h>    // bus I2C (moteurs)
#include <Servo.h>   // pilotage du servomoteur portant l'ultrason

// ============================================
// BROCHES
// ============================================
#define PIN_SERVO     6  
#define PIN_ENCODEUR  2   // broche d'interruption pour compter les impulsions
#define PIN_ULTRASON  7

// ============================================
// ADRESSES I2C
// ============================================
#define MOTEUR_G  0x66
#define MOTEUR_D  0x68
#define ARRET    0x00
#define AVANT    0x01
#define ARRIERE  0x02
#define FREIN    0x03

// ============================================
// VARIABLES
// ============================================
Servo servoUltrason;
volatile unsigned long compteurImpulsions = 0;   // modifiée dans l'ISR -> volatile

// Routine d'interruption : incrémentée à chaque front montant de l'encodeur.
void ISR_encodeur() {
  compteurImpulsions++;
}

// ============================================
void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(200);
  
  servoUltrason.attach(PIN_SERVO);
  pinMode(PIN_ENCODEUR, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ENCODEUR), ISR_encodeur, RISING);
  
  // Moteurs à l'arrêt au démarrage
  piloterMoteur(MOTEUR_G, ARRET, 0);
  piloterMoteur(MOTEUR_D, ARRET, 0);
  
  // Menu affiché une fois au démarrage
  Serial.println(F("\n========================================"));
  Serial.println(F("       PROGRAMME DE CALIBRATION         "));
  Serial.println(F("========================================"));
  Serial.println(F("Tape un chiffre dans le moniteur serie :"));
  Serial.println(F(" 1 = Test du servo (balayage G/D/avant)"));
  Serial.println(F(" 2 = Test ultrason monte sur servo"));
  Serial.println(F(" 3 = Calibration ENCODEUR : impulsions/cm"));
  Serial.println(F(" 4 = Calibration ROTATION : impulsions pour 90 deg"));
  Serial.println(F(" 5 = Test ligne droite (10 secondes)"));
  Serial.println(F("========================================\n"));
  
  servoUltrason.write(90);   // servo centré (ultrason vers l'avant)
}

// ============================================
// Lecture d'un caractère au clavier -> lancement du test correspondant
// ============================================
void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '1': testServo();        break;
      case '2': testUltrasonServo(); break;
      case '3': calibrationDistance(); break;
      case '4': calibrationRotation(); break;
      case '5': testLigneDroite();  break;
    }
    while (Serial.available()) Serial.read(); // vider buffer
    Serial.println(F("\n>>> Tape un chiffre (1-5) pour relancer un test\n"));
  }
}

// ============================================
// TEST 1 : SERVO
// Vérifie la correspondance angle servo <-> orientation de l'ultrason.
// ============================================
void testServo() {
  Serial.println(F("\n=== TEST SERVO ==="));
  
  Serial.println(F("Position AVANT (90 deg)"));
  servoUltrason.write(90);
  delay(1000);
  
  Serial.println(F("Position GAUCHE (180 deg)"));
  servoUltrason.write(180);
  delay(1000);
  
  Serial.println(F("Position AVANT (90 deg)"));
  servoUltrason.write(90);
  delay(1000);
  
  Serial.println(F("Position DROITE (0 deg)"));
  servoUltrason.write(0);
  delay(1000);
  
  Serial.println(F("Retour AVANT (90 deg)"));
  servoUltrason.write(90);
  delay(1000);
  
  Serial.println(F("Fin du test servo."));
  Serial.println(F("VERIFIE que :"));
  Serial.println(F(" - 90 = ultrason pointe DEVANT"));
  Serial.println(F(" - 180 = ultrason pointe a GAUCHE du robot"));
  Serial.println(F(" - 0 = ultrason pointe a DROITE du robot"));
  Serial.println(F("Si l'orientation est differente, note les angles corrects."));
}

// ============================================
// TEST 2 : ULTRASON SUR SERVO
// Balayage gauche/avant/droite avec mesure de distance pendant 15 s.
// ============================================
void testUltrasonServo() {
  Serial.println(F("\n=== TEST ULTRASON + SERVO ==="));
  Serial.println(F("Place des obstacles a gauche, droite, devant"));
  Serial.println(F("Mesure pendant 15 secondes"));
  delay(2000);
  
  unsigned long debut = millis();
  while (millis() - debut < 15000) {
    servoUltrason.write(180); // gauche
    delay(400);
    long dG = mesureUltrason();
    
    servoUltrason.write(90);  // avant
    delay(400);
    long dF = mesureUltrason();
    
    servoUltrason.write(0);   // droite
    delay(400);
    long dD = mesureUltrason();
    
    Serial.print(F("G="));
    Serial.print(dG);
    Serial.print(F(" cm   F="));
    Serial.print(dF);
    Serial.print(F(" cm   D="));
    Serial.print(dD);
    Serial.println(F(" cm"));
  }
  
  servoUltrason.write(90);
  Serial.println(F("Fin du test."));
}

// ============================================
// TEST 3 : CALIBRATION DISTANCE (impulsions/cm)
// On avance 3 s, on relève les impulsions, on mesure la distance réelle.
// Le rapport impulsions/distance donne le facteur d'étalonnage.
// ============================================
void calibrationDistance() {
  Serial.println(F("\n=== CALIBRATION DISTANCE ==="));
  Serial.println(F("Le robot va avancer pendant 3 secondes."));
  Serial.println(F("Mesure ensuite la distance parcourue au metre."));
  Serial.println(F("Demarrage dans 3 sec..."));
  delay(3000);
  
  compteurImpulsions = 0;   // remise à zéro avant la mesure
  
  // Avancer (sens INVERSE : robot retourne)
  piloterMoteur(MOTEUR_G, ARRIERE, 40);
  piloterMoteur(MOTEUR_D, AVANT,   40);
  
  delay(3000);
  
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
  
  unsigned long imp = compteurImpulsions;
  Serial.print(F("\n>>> Impulsions comptees : "));
  Serial.println(imp);
  Serial.println(F(">>> Mesure la distance parcourue (en cm) avec une regle"));
  Serial.println(F(">>> Calcul : IMPULSIONS_PAR_CM = "));
  Serial.print(imp);
  Serial.println(F(" / distance_mesuree"));
  Serial.println(F(">>> Note cette valeur pour le programme final"));
}

// ============================================
// TEST 4 : CALIBRATION ROTATION (90 degres)
// Même principe que la distance, mais pour une rotation sur place.
// ============================================
void calibrationRotation() {
  Serial.println(F("\n=== CALIBRATION ROTATION 90 deg ==="));
  Serial.println(F("Le robot va pivoter a droite pendant 1 seconde."));
  Serial.println(F("Mesure l'angle reel parcouru."));
  Serial.println(F("Demarrage dans 3 sec..."));
  delay(3000);
  
  compteurImpulsions = 0;
  
  // Rotation droite (les 2 moteurs dans le meme sens "logique")
  // Pour le robot retourne : 
  //   - moteur G en ARRIERE (recule, sens inverse de marche avant)
  //   - moteur D en ARRIERE
  // Ca fait pivoter le robot a droite (essai-erreur)
  piloterMoteur(MOTEUR_G, ARRIERE, 30);
  piloterMoteur(MOTEUR_D, ARRIERE, 30);
  
  delay(1000);
  
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
  
  unsigned long imp = compteurImpulsions;
  Serial.print(F("\n>>> Impulsions pendant 1 sec de rotation : "));
  Serial.println(imp);
  Serial.println(F(">>> Mesure l'angle parcouru (en degres) au rapporteur"));
  Serial.println(F(">>> Calcul : IMPULSIONS_PAR_90DEG = "));
  Serial.print(imp);
  Serial.println(F(" * 90 / angle_mesure"));
  Serial.println(F(">>> Si le robot a tourne dans le mauvais sens, INVERSE"));
  Serial.println(F("    les directions ARRIERE/AVANT dans la fonction tourneDroite"));
}

// ============================================
// TEST 5 : LIGNE DROITE
// Roule 10 s pour vérifier que les deux moteurs sont bien équilibrés.
// ============================================
void testLigneDroite() {
  Serial.println(F("\n=== TEST LIGNE DROITE ==="));
  Serial.println(F("Demarrage dans 3 sec..."));
  delay(3000);
  
  piloterMoteur(MOTEUR_G, ARRIERE, 40);
  piloterMoteur(MOTEUR_D, AVANT,   40);
  
  delay(10000);
  
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
  
  Serial.println(F("Fin. Le robot doit avoir avance en ligne droite."));
  Serial.println(F("Sinon : les 2 moteurs ne tournent pas a la meme vitesse."));
}

// ============================================
// FONCTIONS BAS NIVEAU
// ============================================

// Mesure de distance par ultrason (méthode trig/echo sur une seule broche).
// Retourne 9999 si aucun écho (timeout).
long mesureUltrason() {
  pinMode(PIN_ULTRASON, OUTPUT);
  digitalWrite(PIN_ULTRASON, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASON, HIGH);
  delayMicroseconds(10);          // impulsion de déclenchement 10 µs
  digitalWrite(PIN_ULTRASON, LOW);
  pinMode(PIN_ULTRASON, INPUT);
  long duree = pulseIn(PIN_ULTRASON, HIGH, 30000UL);
  if (duree == 0) return 9999;
  return duree / 58;              // conversion durée -> cm
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
