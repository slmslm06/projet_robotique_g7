// ============================================================
//  PARCOURS COMPLET v4 - Corrections finales
//   - Arret IMMEDIAT (premier 1111 = frein, confirmation pendant arret)
//   - Recherche ligne CENTREE (s'arrete sur capteurs du milieu)
//   - Tunnel avec ultrason AVANT en continu
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
#define VITESSE_TUNNEL    20    // tres lent pour reactivite
#define CORRECTION        15

// ============================================================
// TRIM CALIBRE
// ============================================================
#define TRIM_G  0
#define TRIM_D  3

// ============================================================
// DETECTIONS
// ============================================================
#define DIST_OBSTACLE       10
#define DIST_MUR_TUNNEL     50
#define DIST_MUR_FRONTAL    18   // mur frontal dans tunnel
#define DIST_MUR_DANGER     12   // ⭐ proximite critique, freinage
#define DIST_INFINIE        9999
#define SEUIL_LIGNE_PERDUE  25

// Bits centraux du capteur de ligne
#define BITS_CENTRAUX  0b0110

// ============================================================
// ARRET LIGNE PERPENDICULAIRE
// ============================================================
#define SEUIL_CONFIRM_ARRIVEE  2  // confirmation pendant arret

// ============================================================
// MANOEUVRES
// ============================================================
#define DUREE_TOURNE_90    880
#define DUREE_PAR_CM       60
#define DIST_RECUL_INIT    10

// ============================================================
// EVITEMENT
// ============================================================
#define DIST_ECART         40
#define DIST_APPROCHE      25
#define DIST_DEPASSEMENT   20
#define DIST_OBST_CIBLE    30
#define DIST_OBST_PROCHE   60
#define DUREE_MAX_LONGER   8000UL

// ============================================================
// TUNNEL
// ============================================================
#define TUNNEL_TIMEOUT          25000UL
#define INTERVAL_SCAN_LATERAL   2000UL  // scan G/D toutes les 2 sec

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

  Serial.println(F("=== PARCOURS COMPLET v4 ==="));
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
      // Verrouillage frein
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
  // ⭐ ARRET LIGNE PERPENDICULAIRE - IMMEDIAT
  // Des la 1ere lecture 1111 : FREIN tout de suite,
  // puis confirmation pendant l'arret.
  // ============================================
  if (e == 0b1111) {
    Serial.println(F(">> 1111 DETECTE - frein immediat"));
    piloterMoteur(MOTEUR_G, FREIN, 0);
    piloterMoteur(MOTEUR_D, FREIN, 0);
    delay(80);  // laisser le robot s'arreter physiquement
    
    // Confirmation : on relit pour s'assurer que ce n'etait pas un glitch
    uint8_t confirmations = 1;  // celle qu'on vient de voir
    for (uint8_t i = 0; i < SEUIL_CONFIRM_ARRIVEE; i++) {
      delay(30);
      uint8_t verif = lireCapteurs();
      Serial.print(F("Verif "));
      Serial.print(i + 1);
      Serial.print(F(" : "));
      Serial.println(verif, BIN);
      if (verif == 0b1111) confirmations++;
    }
    
    if (confirmations >= 2) {
      // Confirmé : ARRIVEE
      Serial.println(F("\n*** LIGNE PERPENDICULAIRE CONFIRMEE - STOP ***"));
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      etatRobot = ARRIVE;
      return;
    } else {
      // Faux positif : on reprend
      Serial.println(F("Faux positif, on reprend"));
      return;  // on revient au loop, prochain cycle reprendra normalement
    }
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
  // Note : 1111 n'est plus traite ici car gere directement
  //        comme "ligne perpendiculaire" dans gererSuiviLigne()
}

// ============================================================
// ⭐ TUNNEL v4 - Ultrason AVANT en continu
// Logique :
//  - Boucle rapide : scan AVANT + verif ligne au sol
//  - Si mur frontal proche : virer vers le cote oppose
//    en CONTINU jusqu'a ce que le mur s'eloigne
//  - Scan G/D occasionnel (toutes les 2s) pour recentrage
// ============================================================
// Etat TUNNEL : traversee guidee par l'ultrason (detail dans le corps).
// On sort de cet etat des que la ligne est retrouvee.
void gererTunnel() {
  Serial.println(F("\n=== TUNNEL v4 ==="));
  unsigned long debutTunnel = millis();
  unsigned long dernierScanLateral = 0;
  long distG = 25, distD = 25;
  bool servoSurAvant = false;
  
  // S'assurer que le servo est devant
  servoUs.write(SERVO_AVANT);
  delay(300);
  servoSurAvant = true;
  
  avancer(VITESSE_TUNNEL, VITESSE_TUNNEL);

  while (millis() - debutTunnel < TUNNEL_TIMEOUT) {

    // --- Mesure AVANT (servo deja sur AVANT) ---
    if (!servoSurAvant) {
      servoUs.write(SERVO_AVANT);
      delay(250);
      servoSurAvant = true;
    }
    long distAvant = mesureUltrason();

    // --- Sortie 1 : ligne retrouvee ---
    uint8_t e = lireCapteurs();
    if (e != 0b0000) {
      Serial.println(F("SORTIE TUNNEL - ligne L2 !"));
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      delay(100);
      etatRobot = SUIVI_LIGNE;
      compteurLignePerdue = 0;
      return;
    }

    // --- DANGER : mur tres proche -> freinage et recul ---
    if (distAvant > 0 && distAvant < DIST_MUR_DANGER) {
      Serial.print(F("DANGER mur a "));
      Serial.println(distAvant);
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      delay(100);
      reculer(VITESSE_TUNNEL, 5 * DUREE_PAR_CM);
    }

    // --- Mur frontal : virage progressif ---
    if (distAvant > 0 && distAvant < DIST_MUR_FRONTAL) {
      Serial.print(F("Mur devant a "));
      Serial.print(distAvant);
      Serial.println(F(" - virage"));
      
      // Determiner le sens : on doit avoir scanne G/D recemment
      // sinon on prend une decision rapide (servo droite, ~300ms)
      if (distG == 25 && distD == 25) {
        // Pas de mesure laterale recente : on en fait une vite
        servoUs.write(SERVO_DROITE);
        delay(250);
        distD = mesureUltrason();
        servoUs.write(SERVO_GAUCHE);
        delay(300);
        distG = mesureUltrason();
        servoUs.write(SERVO_AVANT);
        delay(250);
        servoSurAvant = true;
      }
      
      // Tourner vers le cote ou il y a le plus de place
      // ET continuer tant que le mur frontal est proche
      bool virerGauche = (distG > distD);
      Serial.println(virerGauche ? F("Virage gauche") : F("Virage droite"));
      
      while (distAvant > 0 && distAvant < DIST_MUR_FRONTAL) {
        if (virerGauche) {
          // Pivot doux sur place vers la gauche
          piloterMoteur(MOTEUR_G, AVANT,   VITESSE_TUNNEL);
          piloterMoteur(MOTEUR_D, AVANT,   VITESSE_TUNNEL);
        } else {
          piloterMoteur(MOTEUR_G, ARRIERE, VITESSE_TUNNEL);
          piloterMoteur(MOTEUR_D, ARRIERE, VITESSE_TUNNEL);
        }
        delay(100);
        distAvant = mesureUltrason();
        Serial.print(F("  Pivot - avant="));
        Serial.println(distAvant);
        
        // Securite : maximum 2s de pivot
        if (millis() - debutTunnel > TUNNEL_TIMEOUT) break;
      }
      
      Serial.println(F("Mur degage, reprise avance"));
      // Reset des mesures laterales (plus valides apres rotation)
      distG = 25;
      distD = 25;
      avancer(VITESSE_TUNNEL, VITESSE_TUNNEL);
      continue;
    }

    // --- Scan lateral periodique (recentrage) ---
    if (millis() - dernierScanLateral > INTERVAL_SCAN_LATERAL) {
      dernierScanLateral = millis();
      
      // Pause pour mesurer
      servoUs.write(SERVO_GAUCHE);
      delay(300);
      distG = mesureUltrason();
      servoUs.write(SERVO_DROITE);
      delay(300);
      distD = mesureUltrason();
      servoUs.write(SERVO_AVANT);
      delay(250);
      servoSurAvant = true;
      
      Serial.print(F("Scan - G="));
      Serial.print(distG);
      Serial.print(F("  D="));
      Serial.println(distD);
      
      // --- Sortie 2 : plus de murs lateraux ni devant ---
      long da = mesureUltrason();
      if (distG > DIST_MUR_TUNNEL && distD > DIST_MUR_TUNNEL && da > DIST_MUR_TUNNEL) {
        Serial.println(F("Plus de murs - sortie tunnel"));
        avancerDroit(VITESSE_LENTE, 400);
        chercherLigneCentree(2000);
        etatRobot = SUIVI_LIGNE;
        compteurLignePerdue = 0;
        return;
      }
      
      // Recentrage
      if (distG < DIST_MUR_TUNNEL && distD < DIST_MUR_TUNNEL) {
        long erreur = distG - distD;
        if (abs(erreur) > 8) {
          // Decalage net : petite correction
          if (erreur > 0) {
            // Trop pres mur droit, corriger gauche
            avancer(VITESSE_TUNNEL - 3, VITESSE_TUNNEL + 3);
          } else {
            avancer(VITESSE_TUNNEL + 3, VITESSE_TUNNEL - 3);
          }
          delay(300);
        }
      } else if (distG < DIST_MUR_TUNNEL && distG < 15) {
        // Trop pres mur gauche
        avancer(VITESSE_TUNNEL + 3, VITESSE_TUNNEL - 3);
        delay(300);
      } else if (distD < DIST_MUR_TUNNEL && distD < 15) {
        avancer(VITESSE_TUNNEL - 3, VITESSE_TUNNEL + 3);
        delay(300);
      }
      
      // Reprise avance normale
      avancer(VITESSE_TUNNEL, VITESSE_TUNNEL);
    }

    delay(60);  // boucle rapide : scan frontal toutes les ~100ms
  }

  Serial.println(F("Timeout tunnel"));
  servoUs.write(SERVO_AVANT);
  delay(200);
  etatRobot = SUIVI_LIGNE;
  compteurLignePerdue = 0;
}

// ============================================================
// EVITEMENT GAUCHE v4
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

  // ⭐ Recherche ligne CENTREE
  Serial.println(F("8. Recherche ligne L3 (centree)"));
  if (chercherLigneCentree(5000)) {
    Serial.println(F("Ligne L3 retrouvee centree !"));
  } else {
    Serial.println(F("Timeout"));
  }

  Serial.println(F("9. Pivot 90 gauche pour aligner"));
  tournerGauche(DUREE_TOURNE_90);

  nbObstaclesEvites++;
  etatRobot = SUIVI_LIGNE;
  compteurLignePerdue = 0;
}

// ============================================================
// EVITEMENT DROITE v4
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

  Serial.println(F("8. Recherche ligne L5 (centree)"));
  if (chercherLigneCentree(5000)) {
    Serial.println(F("Ligne L5 retrouvee centree !"));
  } else {
    Serial.println(F("Timeout"));
  }

  Serial.println(F("9. Pivot 90 droite pour aligner"));
  tournerDroite(DUREE_TOURNE_90);

  nbObstaclesEvites++;
  etatRobot = SUIVI_LIGNE;
  compteurLignePerdue = 0;
}

// ============================================================
// LONGER OBSTACLE (idem v3)
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

// ============================================================
// ⭐ RECHERCHE LIGNE CENTREE
// Avance tant que la ligne n'est pas sous le centre du robot.
// Etape 1 : avancer jusqu'a detecter n'importe quel capteur
// Etape 2 : continuer doucement jusqu'a ce que le centre 
//           (bit 1 OU bit 2 = 0b0110) detecte la ligne
// ============================================================
// Comme chercherLigne, mais ne s'arrete que lorsque la ligne se trouve
// sous les capteurs centraux (alignement plus precis).
bool chercherLigneCentree(unsigned long timeoutMs) {
  unsigned long debut = millis();
  
  // Etape 1 : trouver la ligne avec n'importe quel capteur
  avancer(VITESSE_LENTE, VITESSE_LENTE);
  bool premiereDetection = false;
  while (millis() - debut < timeoutMs) {
    uint8_t e = lireCapteurs();
    if (e != 0b0000) {
      Serial.print(F("Premiere detection : "));
      Serial.println(e, BIN);
      premiereDetection = true;
      break;
    }
    delay(15);
  }
  
  if (!premiereDetection) {
    piloterMoteur(MOTEUR_G, FREIN, 0);
    piloterMoteur(MOTEUR_D, FREIN, 0);
    return false;
  }
  
  // Etape 2 : continuer DOUCEMENT jusqu'a ce que les capteurs
  // centraux voient la ligne
  // On accepte : 0110, 0111, 1110, 0100, 0010 (centre engage)
  avancer(VITESSE_LENTE - 8, VITESSE_LENTE - 8);  // tres lent
  unsigned long debutCentrage = millis();
  
  while (millis() - debutCentrage < 1500) {  // max 1.5 sec
    uint8_t e = lireCapteurs();
    // Le centre est engage si au moins un des 2 bits centraux est a 1
    if ((e & BITS_CENTRAUX) != 0) {
      Serial.print(F("Centre engage : "));
      Serial.println(e, BIN);
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      delay(50);
      return true;
    }
    delay(15);
  }
  
  // Timeout du centrage : on s'arrete quand meme, ligne deja detectee
  Serial.println(F("Timeout centrage, on s'arrete"));
  piloterMoteur(MOTEUR_G, FREIN, 0);
  piloterMoteur(MOTEUR_D, FREIN, 0);
  return true;
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
