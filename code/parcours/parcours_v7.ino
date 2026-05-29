// ============================================================
//  PARCOURS COMPLET v7
//   - TUNNEL : scan AVANT a chaque cycle, suivi mur droit
//     cible CENTREE entre les deux murs
//   - EVITEMENT : pivot d'alignement final + drapeau
//     post-evitement actif pendant 5 secondes
// ============================================================

#include <Wire.h>
#include <Servo.h>

#define PIN_SERVO     6
#define PIN_ULTRASON  4

#define MOTEUR_G     0x66
#define MOTEUR_D     0x68
#define ADDR_LF      0x20

#define ARRET    0x00
#define AVANT    0x01
#define ARRIERE  0x02
#define FREIN    0x03
#define REG_DIGITAL  0x07

#define SERVO_AVANT   90
#define SERVO_GAUCHE  180
#define SERVO_DROITE  0

#define VITESSE_BASE      40
#define VITESSE_LENTE     30
#define VITESSE_TUNNEL    18    // ⭐ encore plus lent
#define CORRECTION        15

#define TRIM_G  -3
#define TRIM_D   0

#define DIST_OBSTACLE      10
#define DIST_MUR_TUNNEL    80
#define DIST_MUR_FRONTAL   25
#define DIST_INFINIE       9999
#define SEUIL_LIGNE_PERDUE 25

#define DUREE_TOURNE_90    880
#define DUREE_TOURNE_45    440
#define DUREE_PAR_CM       60
#define DIST_RECUL_INIT    10

#define DIST_ECART         40
#define DIST_APPROCHE      25
#define DIST_DEPASSEMENT   35
#define DIST_OBST_CIBLE    30
#define DIST_OBST_PROCHE   60
#define DUREE_MAX_LONGER   8000UL

// ============================================================
// ⭐ TUNNEL v7 - parametres
// Tunnel = 50 cm large, robot ~18 cm
// Centre = (50 - 18) / 2 = 16 cm du mur
// On vise 16-17 cm du mur droit pour etre centre
// ============================================================
#define TUNNEL_TIMEOUT          30000UL
#define MUR_DIST_CIBLE          15    // cm : centre du tunnel
#define MUR_DIST_TOLERANCE      3
#define MUR_DIST_MAX_VALIDE     55

// ============================================================
// ⭐ POST-EVITEMENT par DUREE (plus fiable que par etat capteur)
// ============================================================
#define DUREE_POST_EVITEMENT    15000UL  // 5 secondes

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

// ⭐ Drapeau par timer
unsigned long debutPostEvitement = 0;

Servo servoUs;

// ============================================================
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
  // Fin du post-evitement
  debutPostEvitement = 0;
  Serial.println(F(">> Fin post-evitement, detection arrivee REACTIVEE"));
  return false;
}

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

  Serial.println(F("=== PARCOURS COMPLET v7 ==="));
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
// SUIVI DE LIGNE
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
  // ARRET LIGNE PERPENDICULAIRE
  // Desactive pendant 5 sec apres un evitement
  // ============================================
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

  // Detection obstacle
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
// ⭐ TUNNEL v7 - SUPER SAFE
// Strategie : avancer par petits pas SECURISES
//   1) Arret total
//   2) Scan AVANT
//   3) Si mur proche devant -> scan G/D pour decider du virage
//   4) Sinon : scan rapide G/D pour le centrage
//   5) Avancer un petit pas avec les corrections
//   6) Boucle
// ============================================================
// Etat TUNNEL : traversee guidee par l'ultrason (detail dans le corps).
// On sort de cet etat des que la ligne est retrouvee.
void gererTunnel() {
  Serial.println(F("\n=== TUNNEL v7 - mode SAFE ==="));
  unsigned long debutTunnel = millis();

  while (millis() - debutTunnel < TUNNEL_TIMEOUT) {

    // ============ 1) ARRET TOTAL ============
    piloterMoteur(MOTEUR_G, FREIN, 0);
    piloterMoteur(MOTEUR_D, FREIN, 0);
    delay(50);

    // ============ 2) Verifier ligne au sol ============
    uint8_t e = lireCapteurs();
    if (e != 0b0000) {
      Serial.println(F("LIGNE TROUVEE - sortie tunnel"));
      delay(100);
      servoUs.write(SERVO_AVANT);
      delay(200);
      etatRobot = SUIVI_LIGNE;
      compteurLignePerdue = 0;
      return;
    }

    // ============ 3) SCAN AVANT ============
    servoUs.write(SERVO_AVANT);
    delay(300);
    long distAvant = mesureUltrason();

    // ============ 4) SCAN GAUCHE ============
    servoUs.write(SERVO_GAUCHE);
    delay(350);
    long distG = mesureUltrason();

    // ============ 5) SCAN DROITE ============
    servoUs.write(SERVO_DROITE);
    delay(400);
    long distD = mesureUltrason();

    Serial.print(F("Tunnel - F="));
    Serial.print(distAvant);
    Serial.print(F("  G="));
    Serial.print(distG);
    Serial.print(F("  D="));
    Serial.println(distD);

    // ============ 6) Sortie : plus de murs ============
    if (distG > DIST_MUR_TUNNEL && distD > DIST_MUR_TUNNEL && distAvant > DIST_MUR_TUNNEL) {
      Serial.println(F("Plus de murs - sortie"));
      servoUs.write(SERVO_AVANT);
      delay(200);
      avancerDroit(VITESSE_LENTE, 400);
      chercherLigne(2500);
      etatRobot = SUIVI_LIGNE;
      compteurLignePerdue = 0;
      return;
    }

    // ============ 7) MUR DEVANT : virage ============
    if (distAvant > 0 && distAvant < DIST_MUR_FRONTAL) {
      Serial.println(F("MUR DEVANT - virage"));
      bool virerGauche = (distG > distD);
      Serial.println(virerGauche ? F("  > virage GAUCHE") : F("  > virage DROITE"));
      
      // Pivot jusqu'a degager le mur (verification continue)
      servoUs.write(SERVO_AVANT);
      delay(250);
      
      unsigned long debutVirage = millis();
      while (true) {
        if (virerGauche) {
          piloterMoteur(MOTEUR_G, AVANT, VITESSE_TUNNEL);
          piloterMoteur(MOTEUR_D, AVANT, VITESSE_TUNNEL);
        } else {
          piloterMoteur(MOTEUR_G, ARRIERE, VITESSE_TUNNEL);
          piloterMoteur(MOTEUR_D, ARRIERE, VITESSE_TUNNEL);
        }
        delay(80);
        long da = mesureUltrason();
        Serial.print(F("  pivot avant="));
        Serial.println(da);
        if (da > DIST_MUR_FRONTAL + 15) break;
        if (millis() - debutVirage > 2500) break;
      }
      
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      delay(100);
      continue;  // recommence le cycle complet
    }

    // ============ 8) AVANCE AVEC RECENTRAGE ============
    // On est centre si distG ~ distD (les deux entre 10 et 50)
    // Sinon corriger
    int vG = VITESSE_TUNNEL;
    int vD = VITESSE_TUNNEL;
    
    if (distG < DIST_MUR_TUNNEL && distD < DIST_MUR_TUNNEL) {
      // Les deux murs visibles : viser le centre
      long erreur = distG - distD;
      if (abs(erreur) > 4) {
        if (erreur > 0) {
          // Plus pres du mur droit -> corriger vers la gauche
          vG -= 3;
          vD += 3;
          Serial.println(F("  correction G"));
        } else {
          // Plus pres du mur gauche -> corriger vers la droite
          vG += 3;
          vD -= 3;
          Serial.println(F("  correction D"));
        }
      } else {
        Serial.println(F("  centre"));
      }
    } else if (distG < DIST_MUR_TUNNEL) {
      // Seul le mur gauche visible : se maintenir a ~17cm
      if (distG < 12) {
        vG += 3;
        vD -= 3;
        Serial.println(F("  mur G trop pres -> ecarter"));
      } else if (distG > 22) {
        vG -= 3;
        vD += 3;
        Serial.println(F("  mur G trop loin -> rapprocher"));
      }
    } else if (distD < DIST_MUR_TUNNEL) {
      // Seul le mur droit visible
      if (distD < 12) {
        vG -= 3;
        vD += 3;
        Serial.println(F("  mur D trop pres -> ecarter"));
      } else if (distD > 22) {
        vG += 3;
        vD -= 3;
        Serial.println(F("  mur D trop loin -> rapprocher"));
      }
    }
    
    // Avancer un petit PAS (300ms environ)
    avancer(vG, vD);
    delay(300);
    // Apres le pas, on revient en haut de la boucle (arret + nouveau scan)
  }

  Serial.println(F("Timeout tunnel"));
  servoUs.write(SERVO_AVANT);
  delay(200);
  etatRobot = SUIVI_LIGNE;
  compteurLignePerdue = 0;
}

// ============================================================
// EVITEMENT GAUCHE v7 - avec PIVOT FINAL D'ALIGNEMENT
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

  Serial.println(F("3. Ecart"));
  avancerDroit(VITESSE_LENTE, DIST_ECART * DUREE_PAR_CM);

  Serial.println(F("4. Pivot 90 droite"));
  tournerDroite(DUREE_TOURNE_90);

  Serial.println(F("5a. Approche aveugle"));
  avancerDroit(VITESSE_LENTE, DIST_APPROCHE * DUREE_PAR_CM);

  Serial.println(F("5b. Longement obstacle"));
  servoUs.write(SERVO_DROITE);
  delay(400);
  longerObstacleParDroite();

  Serial.println(F("6. Depassement"));
  servoUs.write(SERVO_AVANT);
  delay(200);
  avancerDroit(VITESSE_LENTE, DIST_DEPASSEMENT * DUREE_PAR_CM);

  Serial.println(F("7. Pivot 90 droite vers la ligne"));
  tournerDroite(DUREE_TOURNE_90);

  Serial.println(F("8. Recherche ligne"));
  if (chercherLigne(5000)) {
    Serial.println(F("Ligne trouvee"));
    // ⭐ PIVOT FINAL D'ALIGNEMENT (45 degres vers la droite)
    // Pour s'aligner dans le bon sens, vers la suite du parcours
    Serial.println(F("9. Pivot 45 droite pour s'aligner"));
    tournerDroite(DUREE_TOURNE_45);
  } else {
    Serial.println(F("Timeout"));
  }

  // ⭐ Demarrer le timer post-evitement
  debutPostEvitement = millis();
  Serial.println(F(">> POST-EVITEMENT actif pour 5 secondes"));
  
  nbObstaclesEvites++;
  etatRobot = SUIVI_LIGNE;
  compteurLignePerdue = 0;
}

// ============================================================
// EVITEMENT DROITE v7
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

  Serial.println(F("3. Ecart"));
  avancerDroit(VITESSE_LENTE, DIST_ECART * DUREE_PAR_CM);

  Serial.println(F("4. Pivot 90 gauche"));
  tournerGauche(DUREE_TOURNE_90);

  Serial.println(F("5a. Approche aveugle"));
  avancerDroit(VITESSE_LENTE, DIST_APPROCHE * DUREE_PAR_CM);

  Serial.println(F("5b. Longement obstacle"));
  servoUs.write(SERVO_GAUCHE);
  delay(400);
  longerObstacleParGauche();

  Serial.println(F("6. Depassement"));
  servoUs.write(SERVO_AVANT);
  delay(200);
  avancerDroit(VITESSE_LENTE, DIST_DEPASSEMENT * DUREE_PAR_CM);

  Serial.println(F("7. Pivot 90 gauche vers la ligne"));
  tournerGauche(DUREE_TOURNE_90);

  Serial.println(F("8. Recherche ligne"));
  if (chercherLigne(5000)) {
    Serial.println(F("Ligne trouvee"));
    // ⭐ Pour O2, on s'aligne vers la GAUCHE (vers la suite : zone couleur)
    Serial.println(F("9. Pivot 45 gauche pour s'aligner"));
    tournerGauche(DUREE_TOURNE_45);
  } else {
    Serial.println(F("Timeout"));
  }

  debutPostEvitement = millis();
  Serial.println(F(">> POST-EVITEMENT actif pour 5 secondes"));
  
  nbObstaclesEvites++;
  etatRobot = SUIVI_LIGNE;
  compteurLignePerdue = 0;
}

// ============================================================
// LONGER OBSTACLE
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
