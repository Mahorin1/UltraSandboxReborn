# Prototype 14 corps (bassin + colonne)

Prototype isolé, **non intégré au jeu** : 14 corps / 13 joints (bassin, colonne,
poitrine, tête, 4 bras, 4 jambes, 2 pieds) + contrôleur (IK de jambe, stand/walk/
run/crouch, machine de relève).

## Lancer

```sh
make -C prototype clean test   # tests unitaires + simulation headless
make -C prototype run          # démo jouable (Raylib)
```

- `test_pelvis14` : modèle, IK, démarches, machine de relève (sans physique).
- `test_sim_pelvis14` : monde Box2D headless, sol statique, capteurs d'appui.
- `demo_pelvis14` : fenêtre Raylib, 14 corps rendus, contrôles clavier
  (A/D marcher, Shift courir, C accroupi, R relever, S ralenti, F1 debug).
  Si `pkg-config` ne trouve pas Box2D/Raylib : exporter le `PKG_CONFIG_PATH`
  du projet (`export PKG_CONFIG_PATH=/tmp/us-deps/prefix/lib/pkgconfig`).

## Conventions et pièges

- Le prototype est authoré **y vers le haut** (tête 1.30 > bassin 0.72 > pieds 0.34),
  à l'inverse du jeu (y vers le bas) : la gravité du harness est donc `(0, -9.8)`.
  Une intégration au jeu demandera une conversion d'axe.
- Sol : dessus à `y = 0.32` ; pieds au repos dessus ; capteurs cercle `r = 0.08` sur
  les pieds, les avant-bras et les tibias.
- Box2D 3.2 n'émet un événement de capteur que si **les deux** formes autorisent
  `enableSensorEvents` (`src/solver.c`, `src/sensor.c`) : le sol doit l'activer
  aussi, sinon aucun contact n'est rapporté.
- Longueurs d'os réelles : cuisse 0.12, tibia 0.20 (hanche 0.72, genou 0.60,
  cheville 0.40). L'IK doit utiliser ces longueurs, pas des valeurs arbitraires.

## Réel vs assisté

- **Réel** : les 13 corps non racine sont dynamiques, contacts pieds/sol par
  capteurs, moteurs de jambe pilotés par l'IK, relève déclenchée sur front montant
  (maintenir l'entrée ne relance pas la séquence).
- **Assisté** : le bassin est **cinématique**. Sa hauteur et son avance sont
  imposées, avec une vitesse de montée/descente bornée à 1.2 m/s. Le déplacement en
  marche est donc commandé et non produit par la friction : le pied d'appui glisse
  à ~0.25 m/s (mesuré 0.248 m/s, soit la vitesse commandée).
- **Assisté** : pendant la relève, les appuis mains/genoux sont forcés
  (`force_hands` / `force_knees`) ; le bassin monte de 0.40 à 0.72 m en ~1.0 s.

## Résultats mesurés

```
sim idle:  y=0.720 angle=0.000 foot=1
sim walk:  dx=0.600 y=0.720 angle=0.000 foot=1 slip=0.248 m/s
sim run:   dx=1.080 y=0.720 foot=1
sim crouch: low=0.550 stand=0.720 angle=0.000
sim getup: state=4 gait=0 y=0.720 foot=1 done_at=1.01s
```

## Comparaison avec le ragdoll actuel (`tests/headless.c`)

| Métrique | Ragdoll actuel | Prototype 14 corps |
|---|---|---|
| Corps / joints | 12 / 11 | 14 / 13 |
| Racine | dynamique | cinématique (assistée) |
| Taille debout | ~2.48 m (159 px) | ~1.10 m |
| Gravité / pas | 18 m/s², 80 Hz | 9.8 m/s², 100 Hz |
| Repos (référence torse au-dessus du sol) | 1.59 m, 2.1° | 0.72 m (poitrine), 0.0° |
| Marche | 1.06 m en 3 s = 0.354 m/s | 0.60 m en 2.4 s = 0.25 m/s commandé |
| Glissement de l'appui | adhérence (friction) | 0.248 m/s ≈ 99 % de la vitesse |
| Course | 1.31 m en 3 s = 0.438 m/s | 1.08 m en 2.4 s = 0.45 m/s commandé |
| Crouch (baisse) | 24 px = 0.375 m (−23.5 %) | 0.170 m (−23.6 %) |
| Angle en crouch | −8.9° | 0.0° (racine figée) |
| Relevé | depuis couché, ≤5 s, fin 1.59 m / 2.4° | 1.01 s, 0.40→0.72 m, assisté |
| Détection d'appui | `on_ground=1` | `foot=1` (2 capteurs réels) |

Lecture : le contrôleur de jambe et la profondeur de squat reproduisent le
comportement du jeu (−23.5 % contre −23.6 %). En revanche la marche et la relève ne
sont **pas encore comparables** : la racine cinématique fournit le déplacement et la
montée, donc le prototype ne démontre ni propulsion par friction, ni équilibre, ni
relevé musculaire autonome.
