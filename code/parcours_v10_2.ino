// ============================================================
//  PARCOURS COMPLET v10.2
//
//  CHANGEMENTS vs v9 :
//   - TUNNEL : COPIE EXACTE de code_tunnel.ino
//              (servo a droite, suivi du mur droit avec cible 26 cm,
//               correction proportionnelle, sortie sur retour ligne)
//   - SECTIONS 5-6-7 INTEGREES (apres l'evitement O2) :
//       * Section 5 : montee rampe R1 + suivi L5 + approche panneau
//       * Section 6 : detection couleur + clignotement ruban LED 3s
//       * Section 7 : recul, demi-tour gauche, recherche ligne L5
//   - Apres section 7 : suivi de ligne reprend (descente rampe + V4)
//     pour rejoindre la suite du parcours (etape 10).
//
//  CORRECTIONS v10.2 :
//   - O1 : refonte en MIROIR EXACT d'O2 (qui fonctionne bien)
//          * meme distance de depassement (35 cm)
//          * ajout pivot 70 gauche final (symetrique du pivot 70
//            droite d'O2) pour realigner sur L3
//   - O1/O2 : post-evitement active AVANT chercherLigne (pour
//             eviter l'arret faussement detecte comme ligne
//             d'arrivee si le robot tombe perpendiculaire)
//   - Servo : helper "assurerServoAvant()" appele en debut de
//             suivi de ligne pour garantir que le capteur ultrason
//             est toujours en avant quand le robot suit une ligne
//
//  LOGIQUE DU PARCOURS :
//   ATTENTE_DEPART
//      -> SUIVI_LIGNE (section 1 : V1 + L1)
//      -> TUNNEL (section 2)
//      -> SUIVI_LIGNE (section 3 : L2 + V2)
//      -> EVITEMENT_O1 (section 4)
//      -> SUIVI_LIGNE (section 5 partie L3 + V3 + L4)
//      -> EVITEMENT_O2 (section 6)
//      -> SUIVI_LIGNE (section 7 : montee rampe sur L5)
//      -> ZONE_COULEUR (sections 5-6-7 : detection + LED + demi-tour)
//      -> SUIVI_LIGNE (section 7 fin + descente rampe + V4 + L6 + V5 + L7)
//      -> ARRIVE (ligne d'arrivee)
// ============================================================

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

// ============================================================
// BROCHES
// ============================================================
#define PIN_SERVO      6
#define PIN_ULTRASON   4
#define PIN_LED_RUBAN  3

// ============================================================
// ADRESSES I2C
// ============================================================
#define MOTEUR_G     0x66
#define MOTEUR_D     0x68
#define ADDR_LF      0x20
#define ADDR_COL     0x29

#define ARRET    0x00
#define AVANT    0x01
#define ARRIERE  0x02
#define FREIN    0x03

#define REG_DIGITAL   0x07
#define COL_ENABLE    0x00
#define COL_ATIME     0x01
#define COL_CONTROL   0x0F
#define COL_RDATAL    0x16
#define COL_GDATAL    0x18
#define COL_BDATAL    0x1A

// ============================================================
// SERVO
// ============================================================
#define SERVO_AVANT    90
#define SERVO_GAUCHE   180
#define SERVO_DROITE   0

// ============================================================
// VITESSES (echelle 0..63)
// ============================================================
#define VITESSE_BASE         40
#define VITESSE_LENTE        30
#define VITESSE_TRES_LENTE   22
#define CORRECTION           15

#define TRIM_G  -3
#define TRIM_D   0

// ============================================================
// PARAMETRES SUIVI / OBSTACLES
// ============================================================
#define DIST_OBSTACLE      10
#define DIST_MUR_TUNNEL    80
#define DIST_INFINIE       9999
#define SEUIL_LIGNE_PERDUE 25

#define DUREE_TOURNE_180   1760UL
#define DUREE_TOURNE_90    880
#define DUREE_TOURNE_70    680
#define DUREE_TOURNE_45    440
#define DUREE_PAR_CM       60UL
#define DIST_RECUL_INIT    10

#define DIST_ECART         40
#define DIST_APPROCHE      25
#define DIST_DEPASSEMENT   35
#define DIST_DEPASSEMENT1  40
#define DIST_OBST_CIBLE    30
#define DIST_OBST_PROCHE   60
#define DUREE_MAX_LONGER   8000UL

// ============================================================
// TUNNEL - parametres
// ============================================================
// Logique copiee depuis code_tunnel.ino :
// Servo a droite, suivi du mur droit, cible ~26 cm,
// sortie sur retour de la ligne.
// Seul un timeout de securite est ajoute pour ne pas bloquer.
#define TUNNEL_TIMEOUT          30000UL

// ============================================================
// POST-EVITEMENT par DUREE
// Utilise apres chaque evitement d'obstacle ET apres la zone
// couleur, pour empecher une fausse detection de ligne
// perpendiculaire (3+ capteurs noirs) due au passage sur la
// rampe, V4 ou V5.
// ============================================================
#define DUREE_POST_EVITEMENT    15000UL

// ============================================================
// ZONE COULEUR (sections 5-6-7)
// ============================================================
#define DIST_DETECTE_PANNEAU      15
#define SEUIL_FIN_LIGNE_COULEUR   10
#define TIMEOUT_SUIVI_LIGNE_MS    25000UL
#define ULTRASON_CHAQUE_N_CYCLES  3

#define DIST_ARRET_PANNEAU        2
#define DIST_APPROCHE_LENTE       10
#define TIMEOUT_APPROCHE_MS       5000UL

// LEDs : 2 Hz = periode 500 ms = demi-periode 250 ms, sur 3 s = 12 demi-periodes
#define DEMI_PERIODE_LED          250UL
#define NB_DEMI_PERIODES_LED      12
#define NB_LEDS                   30

#define DIST_RETOUR_AVEUGLE_CM    12
#define DUREE_DESCENTE_RAMPE_MS   6000UL

// ============================================================
// MACHINE A ETATS
// ============================================================
enum Etat {
  ATTENTE_DEPART,
  SUIVI_LIGNE,
  TUNNEL,
  EVITEMENT_O1,
  EVITEMENT_O2,
  ZONE_COULEUR,
  ARRIVE
};

enum Couleur { COUL_ROUGE, COUL_VERT, COUL_BLEU };

Etat etatRobot = ATTENTE_DEPART;
uint8_t dernierEtatCapteurs = 0b0110;
uint16_t compteurLignePerdue = 0;
uint8_t nbObstaclesEvites = 0;
bool tunnelFait = false;
bool zoneCouleurFaite = false;

unsigned long debutPostEvitement = 0;

Servo servoUs;
int servoPositionActuelle = SERVO_AVANT;   // tracking de la position commandee
Adafruit_NeoPixel ruban(NB_LEDS, PIN_LED_RUBAN, NEO_GRB + NEO_KHZ800);
Couleur couleurDetectee;

// ============================================================
// UTILITAIRES
// ============================================================
// Met le servo a un angle donne (sans delay si deja a la position)
// Wrapper d'ecriture d'un angle sur le servo de l'ultrason.
void servoEcrire(int angle) {
  if (angle != servoPositionActuelle) {
    servoUs.write(angle);
    servoPositionActuelle = angle;
  }
}

// Force le servo en position AVANT (a appeler avant tout suivi de ligne)
// N'envoie la commande que si necessaire pour eviter le jitter.
// Garantit que l'ultrason est oriente vers l'avant (reprise du suivi de ligne).
void assurerServoAvant() {
  if (servoPositionActuelle != SERVO_AVANT) {
    servoUs.write(SERVO_AVANT);
    servoPositionActuelle = SERVO_AVANT;
    delay(150);
  }
}

// Compte le nombre de bits a 1 (nombre de capteurs voyant la ligne).
uint8_t compterBits(uint8_t v) {
  uint8_t n = 0;
  for (uint8_t i = 0; i < 4; i++) {
    if (v & (1 << i)) n++;
  }
  return n;
}

// Indique si on est encore dans la fenetre de temps suivant un evitement,
// pendant laquelle la detection de la ligne d'arrivee est inhibee.
bool estEnPostEvitement() {
  if (debutPostEvitement == 0) return false;
  if (millis() - debutPostEvitement < DUREE_POST_EVITEMENT) return true;
  debutPostEvitement = 0;
  Serial.println(F(">> Fin post-evitement, detection arrivee REACTIVEE"));
  return false;
}

// ============================================================
// SETUP
// ============================================================
// Initialisation : bus I2C, liaison serie, servo et peripheriques.
// On attend que le robot soit pose sur la ligne avant de demarrer.
void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(100);

  servoUs.attach(PIN_SERVO);
  servoEcrire(SERVO_AVANT);
  delay(500);

  // Init capteur couleur TCS34725
  ecrireRegistreCol(COL_ATIME,   0xF6);
  ecrireRegistreCol(COL_CONTROL, 0x01);
  ecrireRegistreCol(COL_ENABLE,  0x03);
  delay(100);

  // Init ruban LED
  ruban.begin();
  ruban.setBrightness(150);
  eteindreRuban();

  piloterMoteur(MOTEUR_G, ARRET, 0);
  piloterMoteur(MOTEUR_D, ARRET, 0);

  Serial.println(F("=== PARCOURS COMPLET v10_2 ==="));
  Serial.println(F("Place le robot sur la ligne de depart..."));

  while (lireCapteurs() == 0b0000) {
    delay(100);
  }
  Serial.println(F("GO !"));
  delay(500);
  etatRobot = SUIVI_LIGNE;
}

// ============================================================
// LOOP
// ============================================================
// Boucle principale : aiguillage vers le comportement correspondant
// a l'etat courant du robot (machine a etats).
void loop() {
  switch (etatRobot) {
    case SUIVI_LIGNE:    gererSuiviLigne();    break;
    case TUNNEL:         gererTunnel();        break;
    case EVITEMENT_O1:   evitementGauche();    break;
    case EVITEMENT_O2:   evitementDroite();    break;
    case ZONE_COULEUR:   gererZoneCouleur();   break;
    case ARRIVE:
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      delay(500);
      break;
    default: break;
  }
}

// ============================================================
// SUIVI DE LIGNE (inchange par rapport a v9)
// ============================================================
// Etat SUIVI_LIGNE : lit les capteurs puis teste dans l'ordre la ligne
// d'arrivee, un obstacle et l'entree du tunnel ; sinon corrige la trajectoire.
void gererSuiviLigne() {
  // Garantit que le servo / capteur ultrason est toujours en avant
  // pendant le suivi de ligne (pas de jitter si deja en place).
  assurerServoAvant();

  uint8_t e = lireCapteurs();

  if (e != 0b0000) {
    dernierEtatCapteurs = e;
    compteurLignePerdue = 0;
  } else {
    compteurLignePerdue++;
  }

  if (!estEnPostEvitement()) {
    uint8_t nbCapteurs = compterBits(e);
    if (nbCapteurs >= 3) {
      Serial.print(F(">>> "));
      Serial.print(nbCapteurs);
      Serial.print(F(" capteurs noirs ("));
      Serial.print(e, BIN);
      Serial.println(F(") - frein de precaution"));

      piloterMoteur(MOTEUR_G, FREIN, 63);
      piloterMoteur(MOTEUR_D, FREIN, 63);
      delay(150);
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);

      uint8_t verifs[3];
      for (uint8_t i = 0; i < 3; i++) {
        delay(80);
        verifs[i] = lireCapteurs();
        Serial.print(F("Verif "));
        Serial.print(i + 1);
        Serial.print(F("/3 : "));
        Serial.print(verifs[i], BIN);
        Serial.print(F(" ("));
        Serial.print(compterBits(verifs[i]));
        Serial.println(F(" bits)"));
      }

      uint8_t nbVerifsOK = 0;
      for (uint8_t i = 0; i < 3; i++) {
        if (compterBits(verifs[i]) >= 3) nbVerifsOK++;
      }

      if (nbVerifsOK >= 2) {
        Serial.println(F("*** LIGNE PERPENDICULAIRE - STOP ***"));
        etatRobot = ARRIVE;
        return;
      } else {
        Serial.print(F("Intersection ("));
        Serial.print(nbVerifsOK);
        Serial.println(F("/3) - reprise"));
        return;
      }
    }
  } else {
    if (compterBits(e) >= 3) {
      Serial.print(F("(Post-evitement actif, ignore : "));
      Serial.print(e, BIN);
      Serial.print(F(" - reste "));
      Serial.print((DUREE_POST_EVITEMENT - (millis() - debutPostEvitement)) / 1000);
      Serial.println(F("s)"));
    }
  }

  // Detection panneau couleur / obstacles selon phase du parcours :
  //  - Avant le 2e evitement (O2) : detection obstacles O1/O2
  //  - Apres O2 et avant la zone couleur : detection panneau couleur
  //  - Apres la zone couleur : plus de detection (le robot enchaine
  //    descente rampe + V4 + L6 + V5 + L7 + arret sur ligne d'arrivee)
  if (nbObstaclesEvites < 2) {
    // Detection obstacles O1/O2
    long distAvant = mesureUltrason();
    if (distAvant > 0 && distAvant < DIST_OBSTACLE) {
      Serial.print(F("OBSTACLE a "));
      Serial.print(distAvant);
      Serial.println(F(" cm"));
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      delay(200);
      etatRobot = (nbObstaclesEvites == 0) ? EVITEMENT_O1 : EVITEMENT_O2;
      return;
    }
  } else if (!zoneCouleurFaite) {
    // Detection du panneau de couleur sur la rampe / plateforme P1
    long distAvant = mesureUltrason();
    if (distAvant > 0 && distAvant <= DIST_DETECTE_PANNEAU) {
      // Confirmation
      delay(60);
      long verif = mesureUltrason();
      if (verif > 0 && verif <= DIST_DETECTE_PANNEAU + 3) {
        Serial.print(F("PANNEAU COULEUR detecte a "));
        Serial.print(verif);
        Serial.println(F(" cm"));
        piloterMoteur(MOTEUR_G, FREIN, 0);
        piloterMoteur(MOTEUR_D, FREIN, 0);
        delay(150);
        etatRobot = ZONE_COULEUR;
        return;
      }
    }
  }
  // (Apres zoneCouleurFaite : aucune detection ultrason pendant le
  //  suivi de ligne -> on laisse le robot terminer le parcours.)

  // Detection tunnel : seulement avant d'avoir traverse le tunnel
  if (!tunnelFait && compteurLignePerdue > SEUIL_LIGNE_PERDUE) {
    servoEcrire(SERVO_DROITE);
    delay(300);
    long distD = mesureUltrason();
    servoEcrire(SERVO_AVANT);
    delay(300);

    if (distD > 0 && distD < DIST_MUR_TUNNEL) {
      Serial.println(F("ENTREE TUNNEL"));
      etatRobot = TUNNEL;
      return;
    } else {
      compteurLignePerdue = 0;
    }
  } else if (compteurLignePerdue > SEUIL_LIGNE_PERDUE) {
    // Apres le tunnel, ligne perdue = on continue selon le dernier etat
    compteurLignePerdue = 0;
  }

  piloterSelonEtat(e);
  delay(20);
}

// Traduit l'etat des 4 capteurs en consigne moteur (centre, leger
// decalage, virage...). Si la ligne est perdue, on rejoue le dernier etat.
void piloterSelonEtat(uint8_t e) {
  switch (e) {
    case 0b0110:
      avancer(VITESSE_BASE, VITESSE_BASE);
      break;
    case 0b0100:
      avancer(VITESSE_BASE + CORRECTION, VITESSE_BASE - CORRECTION);
      break;
    case 0b1100:
    case 0b1000:
      avancer(VITESSE_BASE, 0);
      break;
    case 0b0010:
      avancer(VITESSE_BASE - CORRECTION, VITESSE_BASE + CORRECTION);
      break;
    case 0b0011:
    case 0b0001:
      avancer(0, VITESSE_BASE);
      break;
    case 0b0000:
      piloterSelonEtat(dernierEtatCapteurs);
      break;
  }
}

// ============================================================
// TUNNEL v10 - COPIE EXACTE de code_tunnel.ino
// (logique : servo a droite, suivi du mur droit, sortie sur ligne)
// ============================================================
// Suivi du mur droit : on maintient une distance de consigne au mur
// grace a l'ultrason oriente a droite (correction tout-ou-rien).
void suivreTunnelMurDroit() {
  servoEcrire(SERVO_DROITE);
  delay(50); // Laisse le temps au servo de rester en place (ou d'y aller)

  long distD = mesureUltrason();

  Serial.print(F("Tunnel distD = "));
  Serial.println(distD);

  // Objectif : rester autour de 26 cm du mur droit
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

// Etat TUNNEL : traversee guidee par l'ultrason (detail dans le corps).
// On sort de cet etat des que la ligne est retrouvee.
void gererTunnel() {
  Serial.println(F("\n=== TUNNEL (suivi mur droit) ==="));
  unsigned long debutTunnel = millis();

  while (millis() - debutTunnel < TUNNEL_TIMEOUT) {
    suivreTunnelMurDroit();

    // Sortie tunnel : on retrouve la ligne
    if (lireCapteurs() != 0b0000) {
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      delay(150);

      servoEcrire(SERVO_AVANT);
      delay(200);

      tunnelFait = true;
      compteurLignePerdue = 0;
      etatRobot = SUIVI_LIGNE;
      Serial.println(F("SORTIE TUNNEL -> RETOUR LIGNE"));
      return;
    }

    delay(30);
  }

  // Timeout de securite
  Serial.println(F("Timeout tunnel"));
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
  servoEcrire(SERVO_AVANT);
  delay(200);
  tunnelFait = true;
  etatRobot = SUIVI_LIGNE;
  compteurLignePerdue = 0;
}

// ============================================================
// EVITEMENT GAUCHE (O1) - MIROIR EXACT de evitementDroite (O2)
// (puisque O2 marche bien, on applique la meme logique en miroir
//  gauche/droite ; en particulier : pivot 70 gauche final pour
//  realigner le robot dans le sens de progression de L3)
// ============================================================
// Evitement de l'obstacle O1 par la gauche : recul, contournement en
// rectangle, longement de l'obstacle a l'ultrason puis realignement.
void evitementGauche() {
  Serial.println(F("\n=== EVITEMENT O1 (gauche) ==="));
  servoEcrire(SERVO_AVANT);
  delay(200);

  Serial.println(F("1. Recul"));
  reculer(VITESSE_LENTE, DIST_RECUL_INIT * DUREE_PAR_CM);

  Serial.println(F("2. Pivot 90 gauche"));
  tournerGauche(DUREE_TOURNE_90);

  Serial.println(F("3. Ecart"));
  avancerDroit(VITESSE_LENTE, DIST_ECART * DUREE_PAR_CM);

  Serial.println(F("4. Pivot 90 droite"));
  tournerDroite(DUREE_TOURNE_90);

  Serial.println(F("5a. Approche aveugle"));
  avancerDroit(VITESSE_LENTE, DIST_APPROCHE * DUREE_PAR_CM);

  Serial.println(F("5b. Longement obstacle"));
  servoEcrire(SERVO_DROITE);
  delay(400);
  longerObstacleParDroite();

  Serial.println(F("6. Depassement"));
  servoEcrire(SERVO_AVANT);
  delay(200);
  avancerDroit(VITESSE_LENTE, DIST_DEPASSEMENT1 * DUREE_PAR_CM);

  Serial.println(F("7. Pivot 90 droite vers la ligne"));
  tournerDroite(DUREE_TOURNE_90);

  // Activer le post-evitement AVANT la recherche de ligne :
  // sinon le robot tombe perpendiculaire a L3 (3+ capteurs noirs)
  // et est interprete a tort comme "ligne d'arrivee".
  debutPostEvitement = millis();
  Serial.println(F(">> POST-EVITEMENT actif"));

  Serial.println(F("8. Recherche ligne"));
  if (chercherLigne(5000)) {
    Serial.println(F("Ligne trouvee"));
    // PIVOT 70 GAUCHE pour s'aligner dans le sens de progression de L3
    // (miroir exact du pivot 70 droite d'O2 qui aligne sur L5)
    Serial.println(F("9. Pivot 70 gauche pour s'aligner sur L3"));
    tournerGauche(DUREE_TOURNE_70);
  } else {
    Serial.println(F("Timeout"));
  }

  nbObstaclesEvites++;
  etatRobot = SUIVI_LIGNE;
  compteurLignePerdue = 0;
}

// ============================================================
// EVITEMENT DROITE (O2) - inchange par rapport a v9
// ============================================================
// Evitement de l'obstacle O2 par la droite (symetrique de l'evitement gauche).
void evitementDroite() {
  Serial.println(F("\n=== EVITEMENT O2 (droite) ==="));
  servoEcrire(SERVO_AVANT);
  delay(200);

  Serial.println(F("1. Recul"));
  reculer(VITESSE_LENTE, DIST_RECUL_INIT * DUREE_PAR_CM);

  Serial.println(F("2. Pivot 90 droite"));
  tournerDroite(DUREE_TOURNE_90);

  Serial.println(F("3. Ecart"));
  avancerDroit(VITESSE_LENTE, DIST_ECART * DUREE_PAR_CM);

  Serial.println(F("4. Pivot 90 gauche"));
  tournerGauche(DUREE_TOURNE_90);

  Serial.println(F("5a. Approche aveugle"));
  avancerDroit(VITESSE_LENTE, DIST_APPROCHE * DUREE_PAR_CM);

  Serial.println(F("5b. Longement obstacle"));
  servoEcrire(SERVO_GAUCHE);
  delay(400);
  longerObstacleParGauche();

  Serial.println(F("6. Depassement"));
  servoEcrire(SERVO_AVANT);
  delay(200);
  avancerDroit(VITESSE_LENTE, DIST_DEPASSEMENT * DUREE_PAR_CM);

  Serial.println(F("7. Pivot 90 gauche vers la ligne"));
  tournerGauche(DUREE_TOURNE_90);

  // Activer le post-evitement AVANT la recherche de ligne
  debutPostEvitement = millis();
  Serial.println(F(">> POST-EVITEMENT actif"));

  Serial.println(F("8. Recherche ligne"));
  if (chercherLigne(5000)) {
    Serial.println(F("Ligne trouvee"));
    Serial.println(F("9. Pivot 70 droite pour s'aligner vers la rampe"));
    tournerDroite(DUREE_TOURNE_70);
  } else {
    Serial.println(F("Timeout"));
  }

  nbObstaclesEvites++;
  etatRobot = SUIVI_LIGNE;
  compteurLignePerdue = 0;
}

// ============================================================
// LONGER OBSTACLE (inchange par rapport a v9)
// ============================================================
// Longe l'obstacle cote droit : attend d'abord de le voir, puis le suit
// a distance constante jusqu'a l'avoir depasse.
void longerObstacleParDroite() {
  unsigned long debut = millis();
  bool obstacleVu = false;
  unsigned long tempsObstaclePerdu = 0;

  while (millis() - debut < DUREE_MAX_LONGER) {
    long dist = mesureUltrason();
    Serial.print(F("Longer D - dist="));
    Serial.println(dist);

    if (!obstacleVu) {
      if (dist < DIST_OBST_PROCHE && dist > 5) {
        Serial.println(F(">> Obstacle vu"));
        obstacleVu = true;
      } else {
        avancer(VITESSE_LENTE, VITESSE_LENTE);
        if (millis() - debut > 2000) {
          Serial.println(F(">> Pas d'obstacle, abort"));
          piloterMoteur(MOTEUR_G, FREIN, 0);
          piloterMoteur(MOTEUR_D, FREIN, 0);
          return;
        }
      }
      delay(50);
      continue;
    }

    if (dist > DIST_OBST_PROCHE) {
      if (tempsObstaclePerdu == 0) tempsObstaclePerdu = millis();
      if (millis() - tempsObstaclePerdu > 300) {
        Serial.println(F(">> Obstacle depasse !"));
        piloterMoteur(MOTEUR_G, FREIN, 0);
        piloterMoteur(MOTEUR_D, FREIN, 0);
        return;
      }
      avancer(VITESSE_LENTE, VITESSE_LENTE);
    } else {
      tempsObstaclePerdu = 0;
      long erreur = dist - DIST_OBST_CIBLE;
      if (abs(erreur) < 5) {
        avancer(VITESSE_LENTE, VITESSE_LENTE);
      } else if (erreur > 0) {
        avancer(VITESSE_LENTE + 3, VITESSE_LENTE - 3);
      } else {
        avancer(VITESSE_LENTE - 3, VITESSE_LENTE + 3);
      }
    }
    delay(50);
  }

  Serial.println(F("Timeout longer"));
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
}

// Idem longerObstacleParDroite, mais cote gauche.
void longerObstacleParGauche() {
  unsigned long debut = millis();
  bool obstacleVu = false;
  unsigned long tempsObstaclePerdu = 0;

  while (millis() - debut < DUREE_MAX_LONGER) {
    long dist = mesureUltrason();
    Serial.print(F("Longer G - dist="));
    Serial.println(dist);

    if (!obstacleVu) {
      if (dist < DIST_OBST_PROCHE && dist > 5) {
        Serial.println(F(">> Obstacle vu"));
        obstacleVu = true;
      } else {
        avancer(VITESSE_LENTE, VITESSE_LENTE);
        if (millis() - debut > 2000) {
          Serial.println(F(">> Pas d'obstacle, abort"));
          piloterMoteur(MOTEUR_G, FREIN, 0);
          piloterMoteur(MOTEUR_D, FREIN, 0);
          return;
        }
      }
      delay(50);
      continue;
    }

    if (dist > DIST_OBST_PROCHE) {
      if (tempsObstaclePerdu == 0) tempsObstaclePerdu = millis();
      if (millis() - tempsObstaclePerdu > 300) {
        Serial.println(F(">> Obstacle depasse !"));
        piloterMoteur(MOTEUR_G, FREIN, 0);
        piloterMoteur(MOTEUR_D, FREIN, 0);
        return;
      }
      avancer(VITESSE_LENTE, VITESSE_LENTE);
    } else {
      tempsObstaclePerdu = 0;
      long erreur = dist - DIST_OBST_CIBLE;
      if (abs(erreur) < 5) {
        avancer(VITESSE_LENTE, VITESSE_LENTE);
      } else if (erreur > 0) {
        avancer(VITESSE_LENTE - 3, VITESSE_LENTE + 3);
      } else {
        avancer(VITESSE_LENTE + 3, VITESSE_LENTE - 3);
      }
    }
    delay(50);
  }

  Serial.println(F("Timeout longer"));
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
}

// ============================================================
// ZONE COULEUR (Sections 5 + 6 + 7)
//   Sequence : approche panneau -> detection -> clignote LED 3s
//              -> recul -> demi-tour 180 gauche -> recherche ligne
//              -> retour en SUIVI_LIGNE (descente rampe + V4)
// ============================================================
// Zone couleur : approche du panneau, lecture de la couleur, clignotement
// du ruban LED pendant 3 s, puis demi-tour et retour sur la ligne.
void gererZoneCouleur() {
  Serial.println(F("\n=== ZONE COULEUR (sections 5-6-7) ==="));

  // -------- PHASE 1 : APPROCHE FINE DU PANNEAU --------
  Serial.println(F("Phase 1 : approche panneau..."));
  bool approcheOK = approcherPanneau();
  if (!approcheOK) {
    Serial.println(F("Timeout approche panneau - on tente quand meme la suite"));
  }
  delay(400); // Stabilisation avant lecture

  // -------- PHASE 2 : LECTURE COULEUR --------
  Serial.println(F("Phase 2 : detection couleur..."));
  couleurDetectee = detecterCouleur();
  Serial.print(F("Couleur detectee : "));
  switch (couleurDetectee) {
    case COUL_ROUGE: Serial.println(F("ROUGE")); break;
    case COUL_VERT:  Serial.println(F("VERT"));  break;
    case COUL_BLEU:  Serial.println(F("BLEU"));  break;
  }
  delay(200);

  // -------- PHASE 3 : CLIGNOTEMENT LED 3 SECONDES (2 Hz) --------
  Serial.println(F("Phase 3 : clignotement LED 3s..."));
  piloterMoteur(MOTEUR_G, ARRET, 0);
  piloterMoteur(MOTEUR_D, ARRET, 0);
  clignoterCouleur(couleurDetectee);

  // -------- PHASE 4 : RECUL POUR DEGAGEMENT --------
  Serial.println(F("Phase 4 : recul pour degagement..."));
  reculer(VITESSE_LENTE, 800);

  // -------- PHASE 5 : DEMI-TOUR 180 GAUCHE --------
  Serial.println(F("Phase 5 : demi-tour 180 gauche..."));
  tournerGauche(DUREE_TOURNE_180);
  delay(200);

  // -------- PHASE 6 : RETOUR AVEUGLE + RECHERCHE LIGNE L5 --------
  Serial.println(F("Phase 6 : retour aveugle puis recherche ligne L5..."));
  avancerDroit(VITESSE_LENTE, DIST_RETOUR_AVEUGLE_CM * DUREE_PAR_CM);
  if (chercherLigne(4000)) {
    Serial.println(F("Ligne L5 retrouvee !"));
  } else {
    Serial.println(F("Timeout recherche ligne"));
  }

  // -------- FIN ZONE COULEUR --------
  zoneCouleurFaite = true;
  servoEcrire(SERVO_AVANT);
  delay(200);

  // Marquer un post-evitement pour eviter une detection prematuree de la
  // ligne d'arrivee sur la rampe (qui n'a pas de ligne perpendiculaire mais
  // peut donner 3+ capteurs noirs sur certaines portions).
  debutPostEvitement = millis();
  Serial.println(F(">> POST-EVITEMENT actif (descente rampe + V4)"));

  Serial.println(F("=== FIN ZONE COULEUR - reprise suivi de ligne ==="));
  etatRobot = SUIVI_LIGNE;
  compteurLignePerdue = 0;
}

// ============================================================
// APPROCHE FINE DU PANNEAU (lentement jusqu'a ~2 cm)
// ============================================================
// Avance vers le panneau jusqu'a la distance de lecture (ultrason).
// Renvoie true si le panneau a bien ete atteint.
bool approcherPanneau() {
  servoEcrire(SERVO_AVANT);
  delay(200);
  unsigned long debut = millis();

  while (millis() - debut < TIMEOUT_APPROCHE_MS) {
    long dist = mesureUltrason();
    if (dist > 0 && dist <= DIST_ARRET_PANNEAU) {
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      delay(80);
      long verif = mesureUltrason();
      if (verif > 0 && verif <= DIST_ARRET_PANNEAU + 2) return true;
    }
    if (dist > 0 && dist < DIST_APPROCHE_LENTE) {
      avancer(VITESSE_TRES_LENTE, VITESSE_TRES_LENTE);
    } else {
      avancer(VITESSE_LENTE, VITESSE_LENTE);
    }
    delay(40);
  }
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
  return false;
}

// ============================================================
// DETECTION COULEUR (TCS34725)
// ============================================================
Couleur detecterCouleur() {
  long sumR = 0, sumG = 0, sumB = 0;
  const int N = 5;

  for (int i = 0; i < N; i++) {
    sumR += lireRegistreCol(COL_RDATAL);
    sumG += lireRegistreCol(COL_GDATAL);
    sumB += lireRegistreCol(COL_BDATAL);
    delay(60);
  }

  int mR = sumR / N;
  int mG = sumG / N;
  int mB = sumB / N;

  Serial.print(F("R=")); Serial.print(mR);
  Serial.print(F(" G=")); Serial.print(mG);
  Serial.print(F(" B=")); Serial.println(mB);

  if (mR > mG && mR > mB) return COUL_ROUGE;
  else if (mG > mR && mG > mB) return COUL_VERT;
  else return COUL_BLEU;
}

// ============================================================
// CLIGNOTEMENT RUBAN LED (2 Hz pendant 3 s)
// ============================================================
// Fait clignoter le ruban dans la couleur detectee (signalisation 3 s).
void clignoterCouleur(Couleur c) {
  uint8_t r = 0, g = 0, b = 0;
  switch (c) {
    case COUL_ROUGE: r = 255; g = 0;   b = 0;   break;
    case COUL_VERT:  r = 0;   g = 255; b = 0;   break;
    case COUL_BLEU:  r = 0;   g = 0;   b = 255; break;
  }

  for (int i = 0; i < NB_DEMI_PERIODES_LED; i++) {
    if (i % 2 == 0) allumerRuban(r, g, b);
    else eteindreRuban();
    delay(DEMI_PERIODE_LED);
  }
  eteindreRuban();
}

// Allume l'ensemble du ruban LED dans la couleur (r, g, b).
void allumerRuban(uint8_t r, uint8_t g, uint8_t b) {
  for (int j = 0; j < NB_LEDS; j++) {
    ruban.setPixelColor(j, ruban.Color(r, g, b));
  }
  ruban.show();
}

// Eteint le ruban LED.
void eteindreRuban() {
  ruban.clear();
  ruban.show();
}

// ============================================================
// I2C CAPTEUR COULEUR
// ============================================================
// Ecrit un octet dans un registre du capteur de couleur (I2C).
void ecrireRegistreCol(byte registre, byte valeur) {
  Wire.beginTransmission(ADDR_COL);
  Wire.write(0x80 | registre);
  Wire.write(valeur);
  Wire.endTransmission();
}

// Lit une valeur 16 bits dans un registre du capteur de couleur (I2C).
int lireRegistreCol(byte registre) {
  Wire.beginTransmission(ADDR_COL);
  Wire.write(0x80 | registre);
  Wire.endTransmission();
  Wire.requestFrom(ADDR_COL, 2);
  return Wire.read() | (Wire.read() << 8);
}

// ============================================================
// MOUVEMENTS PRIMITIFS (inchanges)
// ============================================================
// Avance en ligne droite pendant une duree donnee, puis freine.
void avancerDroit(int vitesse, unsigned long duree) {
  avancer(vitesse, vitesse);
  delay(duree);
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
  delay(50);
}

// Recule pendant une duree donnee, puis freine (sens moteurs inverse).
void reculer(int vitesse, unsigned long duree) {
  piloterMoteur(MOTEUR_G, AVANT,   vitesse);
  piloterMoteur(MOTEUR_D, ARRIERE, vitesse);
  delay(duree);
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
  delay(50);
}

// Rotation sur place vers la gauche pendant la duree donnee.
void tournerGauche(unsigned long duree) {
  piloterMoteur(MOTEUR_G, AVANT, VITESSE_LENTE);
  piloterMoteur(MOTEUR_D, AVANT, VITESSE_LENTE);
  delay(duree);
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
  delay(50);
}

// Rotation sur place vers la droite pendant la duree donnee.
void tournerDroite(unsigned long duree) {
  piloterMoteur(MOTEUR_G, ARRIERE, VITESSE_LENTE);
  piloterMoteur(MOTEUR_D, ARRIERE, VITESSE_LENTE);
  delay(duree);
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
  delay(50);
}

// Avance tout droit jusqu'a retrouver la ligne ou expiration du delai.
// Renvoie true si la ligne a ete retrouvee.
bool chercherLigne(unsigned long timeoutMs) {
  avancer(VITESSE_LENTE, VITESSE_LENTE);
  unsigned long debut = millis();
  while (millis() - debut < timeoutMs) {
    uint8_t e = lireCapteurs();
    if (e != 0b0000) {
      dernierEtatCapteurs = e;
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      delay(50);
      return true;
    }
    delay(15);
  }
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
  return false;
}

// ============================================================
// CAPTEURS / I2C BAS NIVEAU
// ============================================================
// Mesure de distance par ultrason (impulsion -> echo -> conversion cm).
// Renvoie une valeur 'infinie' si aucun echo n'est recu.
long mesureUltrason() {
  pinMode(PIN_ULTRASON, OUTPUT);
  digitalWrite(PIN_ULTRASON, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_ULTRASON, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_ULTRASON, LOW);
  pinMode(PIN_ULTRASON, INPUT);
  long duree = pulseIn(PIN_ULTRASON, HIGH, 30000UL);
  if (duree == 0) return DIST_INFINIE;
  return duree / 58;
}

// Lit le registre numerique du capteur de ligne et renvoie l'etat des
// 4 capteurs sur 4 bits (bit=1 -> sur la ligne).
uint8_t lireCapteurs() {
  Wire.beginTransmission(ADDR_LF);
  Wire.write(REG_DIGITAL);
  if (Wire.endTransmission(false) != 0) return 0;
  Wire.requestFrom(ADDR_LF, (uint8_t)1);
  if (!Wire.available()) return 0;
  uint8_t brut = Wire.read();
  return (~brut) & 0x0F;  // inversion + masque 4 bits : bit=1 = sur la ligne
}

// Marche avant avec trim et bornage de la vitesse. Robot retourne :
// moteur G en ARRIERE et moteur D en AVANT pour avancer.
void avancer(int vitesseG, int vitesseD) {
  vitesseG = vitesseG + TRIM_G;
  vitesseD = vitesseD + TRIM_D;
  vitesseG = constrain(vitesseG, 0, 63);
  vitesseD = constrain(vitesseD, 0, 63);
  piloterMoteur(MOTEUR_G, vitesseG > 0 ? ARRIERE : ARRET, vitesseG);
  piloterMoteur(MOTEUR_D, vitesseD > 0 ? AVANT   : ARRET, vitesseD);
}

// Envoie une consigne (direction + vitesse) a un driver moteur via I2C.
void piloterMoteur(byte adresse, byte direction, byte vitesse) {
  if (vitesse > 63) vitesse = 63;
  byte commande = (vitesse << 2) | direction;  // trame : vitesse (6 bits) + direction (2 bits)
  Wire.beginTransmission(adresse);
  Wire.write(0x00);
  Wire.write(commande);
  Wire.endTransmission();
}
