# Code — Projet robotique G7

Ce dossier contient l'ensemble des programmes Arduino (`.ino`) du robot développé pour le concours de robotique (L3 SPI).

Le robot doit enchaîner de façon autonome un parcours complet : suivi de ligne, traversée d'un tunnel, évitement de deux obstacles, lecture d'une couleur dans une zone dédiée, puis actionnement d'un lanceur. Il est piloté par un Arduino communiquant en **I2C** avec deux drivers moteurs et le capteur suiveur de ligne, et utilise un **capteur ultrason monté sur servomoteur** pour la détection d'obstacles et le suivi de mur.

## Programme principal : `parcours/`

Le cœur du projet est le programme de **parcours complet**, organisé comme une **machine à états** qui enchaîne automatiquement les différentes phases de l'épreuve :

```
ATTENTE_DEPART
   -> SUIVI_LIGNE      (section 1)
   -> TUNNEL           (section 2 : suivi du mur droit à l'ultrason)
   -> SUIVI_LIGNE      (section 3)
   -> EVITEMENT_O1     (section 4 : contournement du 1er obstacle)
   -> SUIVI_LIGNE
   -> EVITEMENT_O2     (section 5 : contournement du 2e obstacle)
   -> SUIVI_LIGNE
   -> ZONE_COULEUR     (sections 6 + 7 : lecture couleur + signalisation LED)
   -> SUIVI_LIGNE      (section 8)
   -> ARRIVE           (ligne d'arrivée)
   -> LANCEUR          (sections 10 + 11 : mesure + tir)
   -> TERMINE
```

Ce programme a été développé de façon **incrémentale** : chaque version corrige ou ajoute un comportement par rapport à la précédente. Toutes les versions sont conservées dans le sous-dossier `parcours/` pour garder l'historique du travail.

| Version | Apport principal |
|---|---|
| `parcours_v3` | Arrêt sur ligne perpendiculaire, évitement avec phase d'approche, tunnel à scan avant. |
| `parcours_v4` | Freinage dès le premier `1111`, recherche de ligne centrée. |
| `parcours_v5` | Détection d'arrivée et sortie d'évitement plus robustes. |
| `parcours_v6` | Tunnel en suivi de mur, détection d'arrivée inhibée après évitement. |
| `parcours_v7` | Tunnel centré entre les murs, alignement post-évitement. |
| `parcours_v8` | Recentrage par pivot systématique dans le tunnel. |
| `parcours_v9` | Refonte du tunnel (mode urgent + scan en biais). |
| `parcours_v10` | Suivi du mur droit + intégration des sections couleur (5-6-7). |
| `parcours_v10_2` | Évitement O1 refait en miroir d'O2, helper servo. |
| `parcours_v10_3` | Entrée tunnel par timer, réglages évitement/servo. |
| **`parcours_vfin`** | **Version finale** : tunnel stabilisé, affichage LCD des sections, séquence lanceur. |

> **Version à utiliser pour la compétition : `parcours_vfin.ino`.** Les autres versions sont conservées à titre d'historique.

## Programmes annexes (tests et calibration)

Ces fichiers, à la racine du dossier `code/`, ont servi à valider chaque sous-système et à calibrer les paramètres avant de les intégrer au programme principal.

| Fichier | Rôle |
|---|---|
| `test_moteur.ino` | Test de base des deux moteurs (avant / arrêt / arrière). |
| `suivi_ligne.ino` | Suivi de ligne seul (brique reprise dans le parcours). |
| `code_tunnel.ino` | Suivi de ligne + traversée du tunnel par suivi du mur droit (brique reprise dans le parcours à partir de la v10). |
| `ultrason_afficheur.ino` | Lecture de la distance ultrason et affichage sur écran LCD. |
| `ruban_LED.ino` | Lecture du capteur de couleur et signalisation par le ruban LED. |
| `calibration.ino` | Menu série : tests servo, ultrason, encodeur (impulsions/cm) et rotation. |
| `test_calibration.ino` | Menu série : calibration des manœuvres (durées d'avance et de rotation, séquences d'évitement). |

## Matériel et connexions

| Élément | Liaison / Broche | Adresse I2C |
|---|---|---|
| Driver moteur gauche | I2C | `0x66` |
| Driver moteur droit | I2C | `0x68` |
| Capteur suiveur de ligne (4 voies) | I2C | `0x20` |
| Capteur de couleur | I2C | `0x29` |
| Servomoteur (orientation ultrason) | D6 | — |
| Capteur ultrason | D4 | — |
| Ruban LED (NeoPixel) | D3 | — |
| Écran LCD RGB | I2C | — |

> **Remarque importante :** le robot est monté « retourné » (grosses roues à l'avant). Pour avancer en ligne droite, le moteur gauche tourne en `ARRIERE` et le moteur droit en `AVANT`. Cette inversion est gérée dans la fonction `avancer()`.

## Constantes à recalibrer

Ces valeurs dépendent du robot et de la piste, et doivent être réajustées après chaque calibration (voir `calibration.ino` et `test_calibration.ino`) :

- `DUREE_TOURNE_90` — durée (ms) pour une rotation de 90°
- `DUREE_PAR_CM` — durée (ms) pour parcourir 1 cm
- `TRIM_G` / `TRIM_D` — correction d'équilibrage des deux moteurs
- `VITESSE_BASE`, `VITESSE_LENTE`, `VITESSE_TUNNEL` — vitesses de consigne

## Utilisation

1. Ouvrir `parcours/parcours_vfin.ino` dans l'IDE Arduino.
2. Installer les bibliothèques nécessaires : `Wire`, `Servo`, `Adafruit_NeoPixel` et `rgb_lcd` (Grove).
3. Régler le moniteur série sur **9600 bauds**.
4. Téléverser, placer le robot sur la ligne de départ et suivre les indications du moniteur série (et de l'écran LCD).
