// ============================================================
//  PARCOURS COMPLET v6
//   - Detection arrivee desactivee apres evitement
//     (le robot doit d'abord retrouver un suivi normal)
//   - TUNNEL : suivi de mur (servo fixe sur un cote)
//     plus fluide et adapte au tunnel ondule
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
#define VITESSE_TUNNEL    20
#define CORRECTION        15

// TRIM CALIBRE
#define TRIM_G  -3
#define TRIM_D   0

// DETECTIONS
#define DIST_OBSTACLE      10
#define DIST_MUR_TUNNEL    50
#define DIST_MUR_FRONTAL   25
#define DIST_MUR_DANGER    12
#define DIST_INFINIE       9999
#define SEUIL_LIGNE_PERDUE 25

// MANOEUVRES
#define DUREE_TOURNE_90    880
#define DUREE_PAR_CM       60
#define DIST_RECUL_INIT    10

// EVITEMENT
#define DIST_ECART         40
#define DIST_APPROCHE      25
#define DIST_DEPASSEMENT   35
#define DIST_OBST_CIBLE    30
#define DIST_OBST_PROCHE   60
#define DUREE_MAX_LONGER   8000UL

// TUNNEL - SUIVI DE MUR DROIT
#define TUNNEL_TIMEOUT          25000UL
#define MUR_DIST_CIBLE          22   // cm : on essaie de maintenir cette distance au mur droit
#define MUR_DIST_TOLERANCE      5    // marge avant correction
#define MUR_DIST_MAX_VALIDE     45   // au-dela, on considere qu'on a perdu le mur

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

// ⭐ Drapeau : on vient de finir un evitement, ne pas detecter l'arrivee
// tant qu'on n'a pas retrouve un suivi de ligne normal (capteur central engage)
bool sortieEvitement = false;

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

  Serial.println(F("=== PARCOURS COMPLET v6 ==="));
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

  // ⭐ Si on sort d'un evitement, attendre d'avoir un suivi
  // de ligne NORMAL (capteur central engage : 0b0110 type)
  // avant de reactiver la detection d'arrivee
  if (sortieEvitement) {
    // L'etat de suivi normal : les capteurs CENTRAUX sont actifs
    // et pas trop d'exterieurs (donc pas 1111, mais 0110, 0100, 0010)
    if ((e & 0b0110) != 0 && compterBits(e) <= 2) {
      Serial.println(F(">> Suivi normal retrouve, detection arrivee REACTIVEE"));
      sortieEvitement = false;
    }
  }

  // ============================================
  // ARRET LIGNE PERPENDICULAIRE
  // (uniquement si on n'est pas en sortie d'evitement)
  // ============================================
  if (!sortieEvitement) {
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
        Serial.print(F("Intersection/virage ("));
        Serial.print(nbVerifsOK);
        Serial.println(F("/3 OK) - reprise"));
        return;
      }
    }
  } else {
    // Mode "post-evitement" : on ignore le risque d'arret
    if (compterBits(e) >= 3) {
      Serial.print(F("(Post-evitement, ignore : "));
      Serial.print(e, BIN);
      Serial.println(F(")"));
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
    // 1111, 1110, 0111 geres dans gererSuiviLigne
  }
}

// ============================================================
// ⭐ TUNNEL v6 - SUIVI DE MUR DROIT
//
// Strategie totalement differente :
//  - Servo FIXE sur SERVO_DROITE pendant toute la traversee
//  - Mesure CONTINUE de la distance au mur droit (ultrason)
//  - Correction proportionnelle pour rester a distance fixe
//  - Pour detecter les murs frontaux (tunnel ondule),
//    on fait un scan AVANT court (~250ms) toutes les 1.5s
//  - Sortie si :
//      * ligne au sol detectee
//      * mur droit perdu (distance > MUR_DIST_MAX_VALIDE)
//        pendant longtemps
// ============================================================
// Etat TUNNEL : traversee guidee par l'ultrason (detail dans le corps).
// On sort de cet etat des que la ligne est retrouvee.
void gererTunnel() {
  Serial.println(F("\n=== TUNNEL v6 - Suivi de mur droit ==="));
  unsigned long debutTunnel = millis();
  unsigned long dernierScanAvant = 0;
  
  // Positionner le servo a droite
  servoUs.write(SERVO_DROITE);
  delay(400);
  
  unsigned long murPerduDepuis = 0;
  
  // Demarrer doucement
  avancer(VITESSE_TUNNEL, VITESSE_TUNNEL);
  
  while (millis() - debutTunnel < TUNNEL_TIMEOUT) {
    
    // --- Sortie 1 : ligne retrouvee ---
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
    
    // --- Mesure du mur droit ---
    long distMurD = mesureUltrason();
    Serial.print(F("Mur D="));
    Serial.print(distMurD);
    
    // --- Sortie 2 : mur perdu durablement ---
    if (distMurD > MUR_DIST_MAX_VALIDE) {
      if (murPerduDepuis == 0) murPerduDepuis = millis();
      if (millis() - murPerduDepuis > 1500) {
        Serial.println(F(" - MUR PERDU - sortie tunnel"));
        piloterMoteur(MOTEUR_G, FREIN, 0);
        piloterMoteur(MOTEUR_D, FREIN, 0);
        delay(100);
        // Avancer un peu puis chercher la ligne
        servoUs.write(SERVO_AVANT);
        delay(300);
        avancerDroit(VITESSE_LENTE, 400);
        chercherLigne(2500);
        etatRobot = SUIVI_LIGNE;
        compteurLignePerdue = 0;
        return;
      }
      // Mur temporairement perdu, on continue tout droit doucement
      avancer(VITESSE_TUNNEL, VITESSE_TUNNEL);
      Serial.print(F(" (mur tempo perdu "));
      Serial.print(millis() - murPerduDepuis);
      Serial.println(F("ms)"));
    } else {
      murPerduDepuis = 0;
      
      // --- Correction proportionnelle ---
      // distance > cible : on est trop loin du mur droit -> tourner D
      // distance < cible : on est trop pres -> tourner G
      long erreur = distMurD - MUR_DIST_CIBLE;
      
      if (abs(erreur) < MUR_DIST_TOLERANCE) {
        // Tout droit
        avancer(VITESSE_TUNNEL, VITESSE_TUNNEL);
        Serial.println(F(" - droit"));
      } else if (erreur > 0) {
        // Trop loin du mur droit (= on a derive vers la gauche)
        // -> ramener le robot vers la droite
        avancer(VITESSE_TUNNEL + 3, VITESSE_TUNNEL - 3);
        Serial.println(F(" - corr droite"));
      } else {
        // Trop pres du mur droit (= on a derive vers la droite)
        // -> ecarter du mur, aller vers la gauche
        avancer(VITESSE_TUNNEL - 3, VITESSE_TUNNEL + 3);
        Serial.println(F(" - corr gauche"));
      }
    }
    
    // --- Scan AVANT periodique pour detecter mur frontal ---
    // Toutes les 1.5 sec, on fait un scan rapide a l'avant
    if (millis() - dernierScanAvant > 1500) {
      dernierScanAvant = millis();
      
      // Stopper momentanement
      piloterMoteur(MOTEUR_G, FREIN, 0);
      piloterMoteur(MOTEUR_D, FREIN, 0);
      
      servoUs.write(SERVO_AVANT);
      delay(300);
      long distAvant = mesureUltrason();
      Serial.print(F("[Scan avant : "));
      Serial.print(distAvant);
      Serial.println(F("]"));
      
      if (distAvant > 0 && distAvant < DIST_MUR_FRONTAL) {
        // Mur frontal proche : on doit virer
        Serial.println(F("MUR FRONTAL - virage gauche"));
        // Dans un tunnel ondule, quand mur frontal,
        // on tourne A GAUCHE (s'eloigner du mur droit suivi)
        unsigned long debutVirage = millis();
        while (true) {
          // Pivot gauche
          piloterMoteur(MOTEUR_G, AVANT, VITESSE_TUNNEL);
          piloterMoteur(MOTEUR_D, AVANT, VITESSE_TUNNEL);
          delay(100);
          long da = mesureUltrason();
          Serial.print(F("  pivot G - avant="));
          Serial.println(da);
          if (da > DIST_MUR_FRONTAL + 15) break;
          if (millis() - debutVirage > 2000) {
            Serial.println(F("Timeout pivot"));
            break;
          }
        }
        piloterMoteur(MOTEUR_G, FREIN, 0);
        piloterMoteur(MOTEUR_D, FREIN, 0);
        delay(100);
      }
      
      // Remettre le servo a droite pour suivi mur
      servoUs.write(SERVO_DROITE);
      delay(300);
      
      // Reprendre l'avance
      avancer(VITESSE_TUNNEL, VITESSE_TUNNEL);
    }
    
    delay(80);  // pause entre 2 mesures du mur droit
  }
  
  Serial.println(F("Timeout tunnel"));
  servoUs.write(SERVO_AVANT);
  delay(200);
  etatRobot = SUIVI_LIGNE;
  compteurLignePerdue = 0;
}

// ============================================================
// EVITEMENT GAUCHE
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

  Serial.println(F("8. Recherche ligne (retour direct suivi)"));
  if (chercherLigne(5000)) {
    Serial.println(F("Ligne trouvee - retour SUIVI LIGNE"));
  } else {
    Serial.println(F("Timeout - retour SUIVI LIGNE"));
  }

  // ⭐ Activer le mode post-evitement
  sortieEvitement = true;
  Serial.println(F(">> Mode POST-EVITEMENT actif : detection arrivee DESACTIVEE"));
  
  nbObstaclesEvites++;
  etatRobot = SUIVI_LIGNE;
  compteurLignePerdue = 0;
}

// ============================================================
// EVITEMENT DROITE
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

  Serial.println(F("8. Recherche ligne (retour direct suivi)"));
  if (chercherLigne(5000)) {
    Serial.println(F("Ligne trouvee - retour SUIVI LIGNE"));
  } else {
    Serial.println(F("Timeout - retour SUIVI LIGNE"));
  }

  sortieEvitement = true;
  Serial.println(F(">> Mode POST-EVITEMENT actif"));
  
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
// PILOTAGE MOTEURS
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
