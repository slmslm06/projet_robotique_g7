// ============================================================
//  PARCOURS COMPLET v3
//  Corrections critiques :
//   - Arret IMMEDIAT sur ligne perpendiculaire (sans condition)
//   - Evitement avec phase d'approche pour caler l'ultrason
//   - Tunnel avec scan AVANT pour anticiper les murs
// ============================================================

#include <Wire.h>
#include <Servo.h>

// ============================================================
// BROCHES
// ============================================================
#define PIN_SERVO     6
#define PIN_ULTRASON  4

// ============================================================
// ADRESSES I2C
// ============================================================
#define MOTEUR_G     0x66
#define MOTEUR_D     0x68
#define ADDR_LF      0x20

#define ARRET    0x00
#define AVANT    0x01
#define ARRIERE  0x02
#define FREIN    0x03
#define REG_DIGITAL  0x07

// ============================================================
// ANGLES SERVO
// ============================================================
#define SERVO_AVANT   90
#define SERVO_GAUCHE  180
#define SERVO_DROITE  0

// ============================================================
// VITESSES
// ============================================================
#define VITESSE_BASE      40
#define VITESSE_LENTE     30
#define VITESSE_TUNNEL    22    // ⭐ tres lent pour eviter les chocs
#define CORRECTION        15

// ============================================================
// TRIM CALIBRE
// ============================================================
#define TRIM_G  -3
#define TRIM_D   0

// ============================================================
// DETECTIONS
// ============================================================
#define DIST_OBSTACLE      10
#define DIST_MUR_TUNNEL    50
#define DIST_MUR_FRONTAL   20    // ⭐ mur devant dans tunnel
#define DIST_INFINIE       9999
#define SEUIL_LIGNE_PERDUE 25

// ============================================================
// ⭐ ARRET LIGNE PERPENDICULAIRE - LOGIQUE SIMPLE
// ============================================================
// Nombre de lectures consecutives 1111 pour valider arrivee
// SEUIL bas (= rapide) car il n'y a pas d'autre 1111 sur le parcours
#define SEUIL_ARRIVEE  3

// ============================================================
// MANOEUVRES (calibrees)
// ============================================================
#define DUREE_TOURNE_90    880
#define DUREE_PAR_CM       60
#define DIST_RECUL_INIT    10

// ============================================================
// ⭐ EVITEMENT - NOUVELLE LOGIQUE EN 3 PHASES
// ============================================================
#define DIST_ECART         40    // ecart lateral
#define DIST_APPROCHE      25    // avance aveugle avant scan lateral
#define DIST_DEPASSEMENT   35   // marge apres obstacle perdu
#define DIST_OBST_CIBLE    30    // distance cible a l'obstacle
#define DIST_OBST_PROCHE   60    // au-dela = pas d'obstacle visible
#define DUREE_MAX_LONGER   8000UL

// ============================================================
// TUNNEL
// ============================================================
#define TUNNEL_TIMEOUT     25000UL

// ============================================================
// MACHINE D'ETATS
// ============================================================
enum Etat {
  ATTENTE_DEPART,
  SUIVI_LIGNE,
  TUNNEL,
  EVITEMENT_O1,
  EVITEMENT_O2,
  ARRIVE
};

Etat etatRobot = ATTENTE_DEPART;
uint8_t dernierEtatCapteurs = 0b0110;
uint8_t compteurArrivee = 0;
uint16_t compteurLignePerdue = 0;
uint8_t nbObstaclesEvites = 0;

Servo servoUs;

// ============================================================
// Initialisation : bus I2C, liaison serie, servo et peripheriques.
// On attend que le robot soit pose sur la ligne avant de demarrer.
void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(100);

  servoUs.attach(PIN_SERVO);
  servoUs.write(SERVO_AVANT);
  delay(500);

  piloterMoteur(MOTEUR_G, ARRET, 0);
  piloterMoteur(MOTEUR_D, ARRET, 0);

  Serial.println(F("=== PARCOURS COMPLET v3 ==="));
  Serial.println(F("Place le robot sur la ligne de depart..."));

  while (lireCapteurs() == 0b0000) {
    delay(100);
  }
  Serial.println(F("GO !"));
  delay(500);
  etatRobot = SUIVI_LIGNE;
}

// ============================================================
// Boucle principale : aiguillage vers le comportement correspondant
// a l'etat courant du robot (machine a etats).
void loop() {
  switch (etatRobot) {
    case SUIVI_LIGNE:    gererSuiviLigne();    break;
    case TUNNEL:         gererTunnel();        break;
    case EVITEMENT_O1:   evitementGauche();    break;
    case EVITEMENT_O2:   evitementDroite();    break;
    case ARRIVE:
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      delay(500);
      break;
    default: break;
  }
}

// ============================================================
// ETAT : SUIVI DE LIGNE
// ============================================================
// Etat SUIVI_LIGNE : lit les capteurs puis teste dans l'ordre la ligne
// d'arrivee, un obstacle et l'entree du tunnel ; sinon corrige la trajectoire.
void gererSuiviLigne() {
  uint8_t e = lireCapteurs();

  if (e != 0b0000) {
    dernierEtatCapteurs = e;
    compteurLignePerdue = 0;
  } else {
    compteurLignePerdue++;
  }

  // ============================================
  // ⭐ ARRET LIGNE PERPENDICULAIRE - SIMPLE
  // 1111 stable = arrivee, point barre
  // ============================================
  if (e == 0b1111) {
    compteurArrivee++;
    Serial.print(F("1111 detecte "));
    Serial.print(compteurArrivee);
    Serial.print(F("/"));
    Serial.println(SEUIL_ARRIVEE);
    
    if (compteurArrivee >= SEUIL_ARRIVEE) {
      Serial.println(F("\n*** LIGNE PERPENDICULAIRE - STOP ***"));
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      delay(100);
      // Double frein pour stop net
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      etatRobot = ARRIVE;
      return;
    }
    
    // Pendant la confirmation : continue tout droit
    avancer(VITESSE_BASE, VITESSE_BASE);
    delay(20);
    return;
  } else {
    compteurArrivee = 0;
  }

  // Detection obstacle devant
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

  // Detection entree tunnel
  if (compteurLignePerdue > SEUIL_LIGNE_PERDUE) {
    servoUs.write(SERVO_DROITE);
    delay(300);
    long distD = mesureUltrason();
    servoUs.write(SERVO_AVANT);
    delay(300);

    if (distD > 0 && distD < DIST_MUR_TUNNEL) {
      Serial.println(F("ENTREE TUNNEL"));
      etatRobot = TUNNEL;
      return;
    } else {
      compteurLignePerdue = 0;
    }
  }

  piloterSelonEtat(e);
  delay(20);
}

// ============================================================
// PILOTAGE SUIVI DE LIGNE
// ============================================================
// Traduit l'etat des 4 capteurs en consigne moteur (centre, leger
// decalage, virage...). Si la ligne est perdue, on rejoue le dernier etat.
void piloterSelonEtat(uint8_t e) {
  switch (e) {
    case 0b0110:
    case 0b0111:
    case 0b1110:
    case 0b1111:
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
// ⭐ TUNNEL v3 - Plus lent + scan AVANT pour anticipation
// ============================================================
// Etat TUNNEL : traversee guidee par l'ultrason (detail dans le corps).
// On sort de cet etat des que la ligne est retrouvee.
void gererTunnel() {
  Serial.println(F("\n=== MODE TUNNEL ==="));
  unsigned long debutTunnel = millis();

  // Cycle de scan : avant, gauche, droite (rotation)
  int phaseScan = 0;  // 0=avant, 1=gauche, 2=droite
  long distAvant = DIST_INFINIE;
  long distG = 25, distD = 25;

  avancer(VITESSE_TUNNEL, VITESSE_TUNNEL);

  while (millis() - debutTunnel < TUNNEL_TIMEOUT) {

    // ⭐ Cycle de scan en 3 positions
    switch (phaseScan) {
      case 0:
        servoUs.write(SERVO_AVANT);
        delay(250);
        distAvant = mesureUltrason();
        break;
      case 1:
        servoUs.write(SERVO_GAUCHE);
        delay(300);
        distG = mesureUltrason();
        break;
      case 2:
        servoUs.write(SERVO_DROITE);
        delay(300);
        distD = mesureUltrason();
        break;
    }
    phaseScan = (phaseScan + 1) % 3;

    Serial.print(F("Tunnel - avant="));
    Serial.print(distAvant);
    Serial.print(F("  G="));
    Serial.print(distG);
    Serial.print(F("  D="));
    Serial.println(distD);

    // ============================================
    // SORTIE 1 : ligne retrouvee
    // ============================================
    uint8_t e = lireCapteurs();
    if (e != 0b0000) {
      Serial.println(F("SORTIE TUNNEL - ligne L2 !"));
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      delay(100);
      servoUs.write(SERVO_AVANT);
      delay(200);
      etatRobot = SUIVI_LIGNE;
      compteurLignePerdue = 0;
      return;
    }

    // ============================================
    // ⭐ MUR DEVANT : virage immediat
    // Le tunnel est ondule, on tourne du cote
    // qui a le plus d'espace
    // ============================================
    if (distAvant > 0 && distAvant < DIST_MUR_FRONTAL) {
      Serial.println(F("MUR DEVANT - virage"));
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      delay(100);
      
      // Choisir le sens du virage : tourne vers le mur le plus eloigne
      if (distG > distD) {
        Serial.println(F("Virage gauche (plus de place)"));
        tournerGauche(300);  // petit virage seulement
      } else {
        Serial.println(F("Virage droite (plus de place)"));
        tournerDroite(300);
      }
      
      // Reset des mesures laterales (elles ne sont plus valides apres virage)
      distG = 25;
      distD = 25;
      avancer(VITESSE_TUNNEL, VITESSE_TUNNEL);
      continue;
    }

    // ============================================
    // SORTIE 2 : plus de murs
    // ============================================
    if (distG > DIST_MUR_TUNNEL && distD > DIST_MUR_TUNNEL && distAvant > DIST_MUR_TUNNEL) {
      Serial.println(F("Plus de murs - sortie"));
      avancerDroit(VITESSE_LENTE, 500);
      servoUs.write(SERVO_AVANT);
      delay(200);
      chercherLigne(2000);
      etatRobot = SUIVI_LIGNE;
      compteurLignePerdue = 0;
      return;
    }

    // ============================================
    // RECENTRAGE entre les deux murs
    // ============================================
    if (distG < DIST_MUR_TUNNEL && distD < DIST_MUR_TUNNEL) {
      long erreur = distG - distD;
      if (abs(erreur) > 5) {
        if (erreur > 0) {
          // Trop pres du mur droit -> corriger vers la gauche
          avancer(VITESSE_TUNNEL - 4, VITESSE_TUNNEL + 4);
        } else {
          // Trop pres du mur gauche -> corriger vers la droite
          avancer(VITESSE_TUNNEL + 4, VITESSE_TUNNEL - 4);
        }
      } else {
        avancer(VITESSE_TUNNEL, VITESSE_TUNNEL);
      }
    } else if (distG < DIST_MUR_TUNNEL) {
      // Un seul mur visible (gauche) -> rester a ~20cm
      if (distG < 15)      avancer(VITESSE_TUNNEL + 4, VITESSE_TUNNEL - 4);
      else if (distG > 30) avancer(VITESSE_TUNNEL - 4, VITESSE_TUNNEL + 4);
      else                 avancer(VITESSE_TUNNEL, VITESSE_TUNNEL);
    } else if (distD < DIST_MUR_TUNNEL) {
      if (distD < 15)      avancer(VITESSE_TUNNEL - 4, VITESSE_TUNNEL + 4);
      else if (distD > 30) avancer(VITESSE_TUNNEL + 4, VITESSE_TUNNEL - 4);
      else                 avancer(VITESSE_TUNNEL, VITESSE_TUNNEL);
    } else {
      avancer(VITESSE_TUNNEL, VITESSE_TUNNEL);
    }
  }

  Serial.println(F("Timeout tunnel"));
  servoUs.write(SERVO_AVANT);
  delay(200);
  etatRobot = SUIVI_LIGNE;
  compteurLignePerdue = 0;
}

// ============================================================
// ⭐ EVITEMENT GAUCHE v3 - Avec phase d'approche
// ============================================================
// Evitement de l'obstacle O1 par la gauche : recul, contournement en
// rectangle, longement de l'obstacle a l'ultrason puis realignement.
void evitementGauche() {
  Serial.println(F("\n=== EVITEMENT O1 (gauche) ==="));
  servoUs.write(SERVO_AVANT);
  delay(200);

  Serial.println(F("1. Recul"));
  reculer(VITESSE_LENTE, DIST_RECUL_INIT * DUREE_PAR_CM);

  Serial.println(F("2. Pivot 90 gauche"));
  tournerGauche(DUREE_TOURNE_90);

  Serial.println(F("3. Ecart 40cm"));
  avancerDroit(VITESSE_LENTE, DIST_ECART * DUREE_PAR_CM);

  Serial.println(F("4. Pivot 90 droite"));
  tournerDroite(DUREE_TOURNE_90);

  // ⭐ ETAPE 5 EN DEUX TEMPS
  Serial.println(F("5a. Approche 25cm en aveugle"));
  avancerDroit(VITESSE_LENTE, DIST_APPROCHE * DUREE_PAR_CM);

  Serial.println(F("5b. Servo droite et longement"));
  servoUs.write(SERVO_DROITE);
  delay(400);
  longerObstacleParDroite();

  Serial.println(F("6. Depassement 20cm"));
  servoUs.write(SERVO_AVANT);
  delay(200);
  avancerDroit(VITESSE_LENTE, DIST_DEPASSEMENT * DUREE_PAR_CM);

  Serial.println(F("7. Pivot 90 droite"));
  tournerDroite(DUREE_TOURNE_90);

  Serial.println(F("3. Ecart 40cm"));
  avancerDroit(VITESSE_LENTE, DIST_ECART * DUREE_PAR_CM);

  Serial.println(F("9. Pivot 90 gauche pour aligner"));
  tournerGauche(DUREE_TOURNE_90);

  nbObstaclesEvites++;
  etatRobot = SUIVI_LIGNE;
  compteurLignePerdue = 0;
}

// ============================================================
// ⭐ EVITEMENT DROITE v3 - Avec phase d'approche
// ============================================================
// Evitement de l'obstacle O2 par la droite (symetrique de l'evitement gauche).
void evitementDroite() {
  Serial.println(F("\n=== EVITEMENT O2 (droite) ==="));
  servoUs.write(SERVO_AVANT);
  delay(200);

  Serial.println(F("1. Recul"));
  reculer(VITESSE_LENTE, DIST_RECUL_INIT * DUREE_PAR_CM);

  Serial.println(F("2. Pivot 90 droite"));
  tournerDroite(DUREE_TOURNE_90);

  Serial.println(F("3. Ecart 40cm"));
  avancerDroit(VITESSE_LENTE, DIST_ECART * DUREE_PAR_CM);

  Serial.println(F("4. Pivot 90 gauche"));
  tournerGauche(DUREE_TOURNE_90);

  Serial.println(F("5a. Approche 25cm en aveugle"));
  avancerDroit(VITESSE_LENTE, DIST_APPROCHE * DUREE_PAR_CM);

  Serial.println(F("5b. Servo gauche et longement"));
  servoUs.write(SERVO_GAUCHE);
  delay(400);
  longerObstacleParGauche();

  Serial.println(F("6. Depassement 20cm"));
  servoUs.write(SERVO_AVANT);
  delay(200);
  avancerDroit(VITESSE_LENTE, DIST_DEPASSEMENT * DUREE_PAR_CM);

  Serial.println(F("7. Pivot 90 gauche"));
  tournerGauche(DUREE_TOURNE_90);

  Serial.println(F("3. Ecart 40cm"));
  avancerDroit(VITESSE_LENTE, DIST_ECART * DUREE_PAR_CM);

  Serial.println(F("9. Pivot 90 droite pour aligner"));
  tournerDroite(DUREE_TOURNE_90);

  nbObstaclesEvites++;
  etatRobot = SUIVI_LIGNE;
  compteurLignePerdue = 0;
}

// ============================================================
// ⭐ LONGER OBSTACLE - Logique amelioree
// 1) D'abord ATTENDRE de voir l'obstacle (max 2 sec)
// 2) Puis longer jusqu'a perdre l'obstacle
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
    
    // PHASE 1 : on attend de voir l'obstacle
    if (!obstacleVu) {
      if (dist < DIST_OBST_PROCHE && dist > 5) {
        Serial.println(F(">> Obstacle vu, debut longement"));
        obstacleVu = true;
      } else {
        // Pas encore l'obstacle, on avance doucement
        avancer(VITESSE_LENTE, VITESSE_LENTE);
        // Timeout : si apres 2s on ne voit toujours rien, on continue droit
        if (millis() - debut > 2000) {
          Serial.println(F(">> Pas d'obstacle visible apres 2s, abort"));
          piloterMoteur(MOTEUR_G, FREIN, 0);
          piloterMoteur(MOTEUR_D, FREIN, 0);
          return;
        }
      }
      delay(50);
      continue;
    }
    
    // PHASE 2 : on suit l'obstacle
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
        // Trop loin -> se rapprocher (tourner D)
        avancer(VITESSE_LENTE + 3, VITESSE_LENTE - 3);
      } else {
        // Trop pres -> s'eloigner (tourner G)
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
        Serial.println(F(">> Obstacle vu, debut longement"));
        obstacleVu = true;
      } else {
        avancer(VITESSE_LENTE, VITESSE_LENTE);
        if (millis() - debut > 2000) {
          Serial.println(F(">> Pas d'obstacle visible apres 2s, abort"));
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
        // Trop loin -> tourner G
        avancer(VITESSE_LENTE - 3, VITESSE_LENTE + 3);
      } else {
        // Trop pres -> tourner D
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
// MOUVEMENTS PRIMITIFS
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
    if (lireCapteurs() != 0b0000) {
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
// LECTURE ULTRASON
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

// ============================================================
// PILOTAGE MOTEURS (avec TRIM)
// ============================================================
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
