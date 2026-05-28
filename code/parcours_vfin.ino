// ============================================================
//  PARCOURS COMPLET v15
//
//  CORRECTIONS v15 :
//   - TUNNEL : retour a la version PARCOURS V10_3
//       * Suppression de l'avance biaisee a l'entree du tunnel
//       * Suppression de DUREE_BIAIS_TUNNEL_MS
//       * Restauration du frein + delay 100 ms avant la bascule
//         vers TUNNEL (transition simple comme v10_3)
//       * gererTunnel() identique a v10_3 (a 3 appels
//         afficherSection() pres pour le LCD, ajouts conserves)
//       * suivreTunnelMurDroit() : inchange (etait deja
//         identique a v10_3 dans v14)
//   - Reste du code v14 : INCHANGE
//
//  RAPPELS :
//   - LANCEUR (sections 10 + 11) : ajoute en v12, DUREE_DC25 40s
//   - Affichage LCD des sections : ajoute en v11
//   - Suivi de ligne avec braquage fort adouci (VITESSE_BASE/3) : v13
//
//  LOGIQUE DU PARCOURS :
//   ATTENTE_DEPART
//      -> SUIVI_LIGNE (section 1)
//      -> TUNNEL (section 2)
//      -> SUIVI_LIGNE (section 3)
//      -> EVITEMENT_O1 (section 4)
//      -> SUIVI_LIGNE (section 4 fin)
//      -> EVITEMENT_O2 (section 5)
//      -> SUIVI_LIGNE (section 5 fin)
//      -> ZONE_COULEUR (sections 6 + 7)
//      -> SUIVI_LIGNE (section 8)
//      -> ARRIVE (ligne d'arrivee)
//      -> LANCEUR (sections 10 + 11 : mesure + tir)
//      -> TERMINE
// ============================================================

#include <Wire.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include "rgb_lcd.h"

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
// Moteurs du lanceur (carte CA + carte C0)
#define ADDR_DC37    0x65    // gros moteur - tend le bras (etiquette CA)
#define ADDR_DC25    0x60    // petit moteur - retire la goupille (etiquette C0)

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
#define CORRECTION           12   // reduit de 15 a 12 pour limiter le zigzag avec coque + lanceur

#define TRIM_G  -3
#define TRIM_D   0

// ============================================================
// PARAMETRES SUIVI / OBSTACLES
// ============================================================
#define DIST_OBSTACLE      10
#define DIST_INFINIE       9999

#define DUREE_TOURNE_180   2000UL
#define DUREE_TOURNE_90    1000
#define DUREE_TOURNE_130   1444
#define DUREE_TOURNE_70    777
#define DUREE_TOURNE_45    500
#define DUREE_PAR_CM       60UL
#define DIST_RECUL_INIT    10

#define DIST_ECART         45
#define DIST_APPROCHE      25
#define DIST_DEPASSEMENT   35   // utilise par O2
#define DIST_DEPASSEMENT1  40   // utilise par O1 (tune sur le terrain)
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
// Detection d'entree de tunnel (comme code_tunnel.ino) :
// timer de 500 ms apres perte totale de la ligne.
#define DUREE_AVANT_TUNNEL_MS   500UL

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
// LANCEUR (sections 10 + 11)
// ============================================================
// Tableau de calibration (butee Haute uniquement) :
//   duree DC37 (s)  ->  distance atteinte (cm)
//        10                70
//        11               100
//        12.5             110
//        15               130
//        17               140
//        20               150
//
// Interpolation lineaire par morceaux entre ces points.
// Plancher 10 s, plafond 20 s.
// Si distance hors plage de mesure (< 50 cm ou > 350 cm) :
// duree par defaut DUREE_DC37_DEFAUT_MS.
#define LANCEUR_NB_POINTS         6
#define DIST_PANIER_MIN_CM        50      // sous ce seuil : mesure invalide
#define DIST_PANIER_MAX_CM        350     // au-dela : mesure invalide
#define DUREE_DC37_DEFAUT_MS      15000UL // duree par defaut si mesure invalide
#define DUREE_DC25_GOUPILLE_MS    60000UL // duree pour retirer la goupille
#define VITESSE_DC37              63
#define VITESSE_DC25              63
#define PAUSE_DC37_DC25_MS        100     // pause courte entre les deux phases

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
  ARRIVE,
  LANCEUR,        // sections 10 + 11 : mesure + tir
  TERMINE         // tout est fini, robot a l'arret definitif
};

enum Couleur { COUL_ROUGE, COUL_VERT, COUL_BLEU };

Etat etatRobot = ATTENTE_DEPART;
uint8_t dernierEtatCapteurs = 0b0110;
uint16_t compteurLignePerdue = 0;
uint8_t nbObstaclesEvites = 0;
bool tunnelFait = false;
bool zoneCouleurFaite = false;

// Detection d'entree de tunnel (logique code_tunnel.ino) :
// timer 500 ms apres perte totale de la ligne.
bool lignePerdueFlag = false;
unsigned long tempsLignePerdue = 0;

unsigned long debutPostEvitement = 0;

Servo servoUs;
int servoPositionActuelle = SERVO_AVANT;   // tracking de la position commandee
Adafruit_NeoPixel ruban(NB_LEDS, PIN_LED_RUBAN, NEO_GRB + NEO_KHZ800);
rgb_lcd lcd;                               // afficheur LCD Grove (I2C)
uint8_t sectionActuelle = 0;               // section actuellement affichee (0 = aucune)
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
// Delay 350 ms suffisant pour un mouvement potentiel de 90 degres.
// Garantit que l'ultrason est oriente vers l'avant (reprise du suivi de ligne).
void assurerServoAvant() {
  if (servoPositionActuelle != SERVO_AVANT) {
    servoUs.write(SERVO_AVANT);
    servoPositionActuelle = SERVO_AVANT;
    delay(350);
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
// AFFICHAGE LCD - Sections du parcours
// ============================================================
// Affiche "Section N" + courte description sur le LCD.
// La sectionActuelle empeche les rafraichissements inutiles
// (et evite le clignotement du backlight si on rappelle plusieurs
// fois avec le meme numero).
// Affiche le numero et le libelle de la section courante sur l'ecran LCD.
void afficherSection(uint8_t numero, const char* description) {
  if (numero == sectionActuelle) return;   // deja affiche, on ne touche pas
  sectionActuelle = numero;

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Section ");
  lcd.print(numero);
  lcd.setCursor(0, 1);
  lcd.print(description);

  Serial.print(F(">>> LCD : Section "));
  Serial.print(numero);
  Serial.print(F(" - "));
  Serial.println(description);
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

  // Init afficheur LCD (16 colonnes, 2 lignes)
  lcd.begin(16, 2);
  lcd.setRGB(0, 128, 255);
  lcd.print("Initialisation");
  delay(500);

  piloterMoteur(MOTEUR_G, ARRET, 0);
  piloterMoteur(MOTEUR_D, ARRET, 0);
  // Securite : moteurs du lanceur a l'arret au demarrage
  piloterMoteur(ADDR_DC37, ARRET, 0);
  piloterMoteur(ADDR_DC25, ARRET, 0);

  Serial.println(F("=== PARCOURS COMPLET vFIN ==="));
  Serial.println(F("Place le robot sur la ligne de depart..."));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Place robot sur");
  lcd.setCursor(0, 1);
  lcd.print("ligne de depart");

  while (lireCapteurs() == 0b0000) {
    delay(100);
  }
  Serial.println(F("GO !"));
  delay(500);

  // Section 1 : virage V1 + ligne L1 vers tunnel
  afficherSection(1, "V1 + L1");
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
      // Frein puis bascule en LANCEUR pour la suite (mesure + tir)
      piloterMoteur(MOTEUR_G, FREIN, 63);
      piloterMoteur(MOTEUR_D, FREIN, 63);
      delay(500);
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      etatRobot = LANCEUR;
      break;
    case LANCEUR:
      gererLanceur();
      break;
    case TERMINE:
      // Tout est termine, on ne fait plus rien
      piloterMoteur(MOTEUR_G, ARRET, 0);
      piloterMoteur(MOTEUR_D, ARRET, 0);
      piloterMoteur(ADDR_DC37, ARRET, 0);
      piloterMoteur(ADDR_DC25, ARRET, 0);
      delay(1000);
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
        // Affichage final : on a passe les sections 8/9/10/11 confondues
        // (le code ne les distingue pas) -> on affiche "Arrivee"
        sectionActuelle = 0;  // force le refresh
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Arrivee !");
        lcd.setCursor(0, 1);
        lcd.print("Ligne franchie");
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

  // ========================================================
  // GESTION DETECTION D'ENTREE DE TUNNEL (logique code_tunnel)
  // Avant le tunnel : on ne fait AUCUNE detection d'obstacle
  // frontal (le seul "obstacle" possible est le mur du tunnel
  // lui-meme, et il faut entrer dedans, pas l'eviter).
  // Detection d'entree = ligne perdue depuis > 500 ms.
  // ========================================================
  if (!tunnelFait) {
    // Mise a jour du timer de perte de ligne
    if (e != 0b0000) {
      lignePerdueFlag = false;
      tempsLignePerdue = 0;
    } else {
      if (!lignePerdueFlag) {
        lignePerdueFlag = true;
        tempsLignePerdue = millis();
      }
    }

    // Si ligne perdue depuis plus de 500 ms : on passe en tunnel
    if (lignePerdueFlag && (millis() - tempsLignePerdue > DUREE_AVANT_TUNNEL_MS)) {
      Serial.println(F("PASSAGE EN TUNNEL (ligne perdue > 500ms)"));
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      delay(100);
      etatRobot = TUNNEL;
      return;
    }

    // Pas encore en tunnel : on continue le suivi de ligne sans
    // detection d'obstacle (sinon le mur du tunnel serait pris
    // pour un obstacle a eviter).
    piloterSelonEtat(e);
    delay(20);
    return;
  }

  // ========================================================
  // APRES LE TUNNEL : detection obstacles ou panneau couleur
  // selon la phase du parcours
  // ========================================================
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
      // Braquage fort gauche : au lieu de 0 (trop brutal -> overshoot),
      // on garde un peu de vitesse sur la roue interieure pour adoucir
      avancer(VITESSE_BASE, VITESSE_BASE / 3);
      break;
    case 0b0010:
      avancer(VITESSE_BASE - CORRECTION, VITESSE_BASE + CORRECTION);
      break;
    case 0b0011:
    case 0b0001:
      // Braquage fort droite : meme principe en miroir
      avancer(VITESSE_BASE / 3, VITESSE_BASE);
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
  afficherSection(2, "Tunnel");
  unsigned long debutTunnel = millis();

  while (millis() - debutTunnel < TUNNEL_TIMEOUT) {
    suivreTunnelMurDroit();

    // Sortie tunnel : on retrouve la ligne
    if (lireCapteurs() != 0b0000) {
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      delay(150);

      // Remise du servo au centre - delay suffisant pour un
      // mouvement de 90 degres (le servo etait a SERVO_DROITE)
      servoEcrire(SERVO_AVANT);
      delay(400);

      tunnelFait = true;
      compteurLignePerdue = 0;
      // Reset du timer de perte de ligne pour proprete
      lignePerdueFlag = false;
      tempsLignePerdue = 0;
      etatRobot = SUIVI_LIGNE;
      Serial.println(F("SORTIE TUNNEL -> RETOUR LIGNE"));
      // Section 3 : L2 + V2 jusqu'a O1
      afficherSection(3, "L2 + V2");
      return;
    }

    delay(30);
  }

  // Timeout de securite
  Serial.println(F("Timeout tunnel"));
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
  servoEcrire(SERVO_AVANT);
  delay(400);
  tunnelFait = true;
  lignePerdueFlag = false;
  tempsLignePerdue = 0;
  etatRobot = SUIVI_LIGNE;
  compteurLignePerdue = 0;
  // Section 3 meme en cas de timeout (on tente de continuer)
  afficherSection(3, "L2 + V2");
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
  // Section 4 : evitement O1 + L3 + V3 + L4
  afficherSection(4, "Evite O1");
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
  avancerDroit(VITESSE_LENTE, DIST_DEPASSEMENT * DUREE_PAR_CM);

  Serial.println(F("7. Pivot 90 droite vers la ligne"));
  tournerDroite(DUREE_TOURNE_130);

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
  // Section 5 : evitement O2 + L5 + rampe R1 + zone P1
  afficherSection(5, "Evite O2");
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
  tournerGauche(DUREE_TOURNE_130);

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
  // Section 6 : allumage du ruban LED de la couleur detectee
  afficherSection(6, "LEDs couleur");
  piloterMoteur(MOTEUR_G, ARRET, 0);
  piloterMoteur(MOTEUR_D, ARRET, 0);
  clignoterCouleur(couleurDetectee);

  // -------- PHASE 4 : RECUL POUR DEGAGEMENT --------
  Serial.println(F("Phase 4 : recul pour degagement..."));
  reculer(VITESSE_LENTE, 1500);

  // -------- PHASE 5 : DEMI-TOUR 180 GAUCHE --------
  Serial.println(F("Phase 5 : demi-tour 180 gauche..."));
  // Section 7 : demi-tour 180 gauche
  afficherSection(7, "Demi-tour");
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
  // Section 8 : descente rampe R1 + V4 + L6 + V5 + L7 + chrono jusqu'a l'arrivee
  // (les sections 9 et 10 ne sont pas distinguees du code -> regroupees ici)
  afficherSection(8, "Chrono + L6");
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
// LANCEUR (sections 10 + 11)
// ============================================================
// Sequence apres arret sur la ligne d'arrivee :
//  1) Mesure la distance au panier (ultrason, servo en avant)
//  2) Affiche "Section 10" + "Dist: X cm" sur le LCD
//  3) Calcule la duree d'activation du DC37 par interpolation
//     lineaire par morceaux dans la table de calibration
//     (plancher 10 s, plafond 20 s, defaut 15 s si hors plage)
//  4) Active le DC37 pendant cette duree (tension du bras)
//  5) Arret bref
//  6) Active le DC25 pendant DUREE_DC25_GOUPILLE_MS pour retirer
//     la goupille -> declenchement du tir
//  7) Tout coupe -> etat TERMINE
// ============================================================

// Calibration : duree DC37 (ms) en fonction de la distance (cm).
// Table croissante en distance.
const int      lanceurDistancesCm[LANCEUR_NB_POINTS] = {  70,  100,  110,  130,  140,  150 };
const long     lanceurDureesMs[LANCEUR_NB_POINTS]    = {10000,11000,12500,15000,17000,20000};

// Interpolation lineaire par morceaux dans la table.
long calculerDureeDC37(long distanceCm) {
  // Hors plage : plancher / plafond
  if (distanceCm <= lanceurDistancesCm[0]) {
    return lanceurDureesMs[0];
  }
  if (distanceCm >= lanceurDistancesCm[LANCEUR_NB_POINTS - 1]) {
    return lanceurDureesMs[LANCEUR_NB_POINTS - 1];
  }
  // Recherche du segment qui encadre distanceCm
  for (int i = 0; i < LANCEUR_NB_POINTS - 1; i++) {
    int d0 = lanceurDistancesCm[i];
    int d1 = lanceurDistancesCm[i + 1];
    if (distanceCm >= d0 && distanceCm <= d1) {
      long t0 = lanceurDureesMs[i];
      long t1 = lanceurDureesMs[i + 1];
      // Interpolation lineaire : t = t0 + (t1 - t0) * (d - d0) / (d1 - d0)
      return t0 + ((t1 - t0) * (distanceCm - d0)) / (d1 - d0);
    }
  }
  // Securite : ne devrait pas arriver
  return DUREE_DC37_DEFAUT_MS;
}

// Sequence finale du lanceur : phase de mesure puis phase de tir.
void gererLanceur() {
  Serial.println(F("\n=== LANCEUR (sections 10 + 11) ==="));

  // -------- 1) Mesure de la distance au panier --------
  // Servo deja en avant (assurerServoAvant fait son boulot dans
  // gererSuiviLigne ; en sortie ARRIVE le servo est en avant).
  // On fait quelques mesures pour stabiliser.
  servoEcrire(SERVO_AVANT);
  delay(300);

  long distances[5];
  for (int i = 0; i < 5; i++) {
    distances[i] = mesureUltrason();
    delay(80);
  }
  // Tri rapide manuel (5 elements, tri par insertion)
  for (int i = 1; i < 5; i++) {
    long key = distances[i];
    int j = i - 1;
    while (j >= 0 && distances[j] > key) {
      distances[j + 1] = distances[j];
      j--;
    }
    distances[j + 1] = key;
  }
  // Mediane = element du milieu
  long distancePanier = distances[2];

  Serial.print(F("Distance panier mesuree : "));
  Serial.print(distancePanier);
  Serial.println(F(" cm"));

  // -------- 2) Affichage LCD : Section 10 + Dist: X cm --------
  // On force le refresh meme si on est encore en "Section 8"
  sectionActuelle = 0;
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Section 10");
  lcd.setCursor(0, 1);
  lcd.print("Dist: ");
  lcd.print(distancePanier);
  lcd.print(" cm");

  // -------- 3) Calcul de la duree DC37 --------
  long dureeDC37;
  if (distancePanier < DIST_PANIER_MIN_CM || distancePanier > DIST_PANIER_MAX_CM) {
    Serial.println(F("Distance hors plage -> duree par defaut"));
    dureeDC37 = DUREE_DC37_DEFAUT_MS;
  } else {
    dureeDC37 = calculerDureeDC37(distancePanier);
  }
  Serial.print(F("Duree DC37 calculee : "));
  Serial.print(dureeDC37);
  Serial.println(F(" ms"));

  // Petite pause avant de commencer le tendage
  delay(300);

  // -------- 4) Tendage du bras (DC37) --------
  Serial.println(F("Tendage DC37..."));
  piloterMoteur(ADDR_DC37, AVANT, VITESSE_DC37);
  delay(dureeDC37);
  // Arret du DC37 (ARRET, pas FREIN, pour ne pas forcer en sens
  // inverse contre la mecanique du bras tendu)
  piloterMoteur(ADDR_DC37, ARRET, 0);
  Serial.println(F("DC37 arrete - bras tendu"));

  // -------- 5) Pause courte avant la goupille --------
  delay(PAUSE_DC37_DC25_MS);

  // -------- 6) Retrait de la goupille (DC25) -> TIR --------
  Serial.println(F("Retrait goupille DC25..."));
  piloterMoteur(ADDR_DC25, AVANT, VITESSE_DC25);
  delay(DUREE_DC25_GOUPILLE_MS);
  piloterMoteur(ADDR_DC25, ARRET, 0);
  Serial.println(F("DC25 arrete - tir effectue"));

  // -------- 7) Termine --------
  // Tous moteurs coupes (securite)
  piloterMoteur(ADDR_DC37, ARRET, 0);
  piloterMoteur(ADDR_DC25, ARRET, 0);
  piloterMoteur(MOTEUR_G, ARRET, 0);
  piloterMoteur(MOTEUR_D, ARRET, 0);

  Serial.println(F("=== FIN LANCEUR ==="));
  etatRobot = TERMINE;
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
