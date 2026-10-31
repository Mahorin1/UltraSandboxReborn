# ml/ — politique neuronale sur le ragdoll d'origine (repo Old)

But : récupérer le ragdoll du repo `UltraSandbox_Old` (C++/SFML2/Box2D v2),
le porter en C/Box2D 3.2, et apprendre une politique qui se déplace à
gauche/droite selon la commande du joueur et s'auto-équilibre.

Tout est **headless** (aucune dépendance à Raylib) et isolé de `src/` : le
ragdoll du jeu n'est pas modifié.

## Ce qui est porté de l'ancien code

| Élément | Valeur (identiques à `ragdoll.cpp`) |
|---|---|
| Corps / joints | 12 / 11 |
| Tête | cercle r = 13.25 px |
| Torse | 11.5 × 43.5 px |
| Bras arrière / avant | 6 × 33 px / 4.8 × 32 px |
| Cuisse / tibia | 8.85 × 39 px / 6.75 × 35.5 px |
| Pied | 15 × 4.33 px |
| Référence cheville | 90° (angle 0 = pied à plat) |
| Frottement | 5 (pieds 20), restitution 0 |
| Filtre | groupe négatif : pas d'auto-collision |
| Capteurs de pied | cercle r = 30 px sur chaque pied |

Différences assumées, nécessaires à une politique unique :

- **limites symétriques** (l'ancien contrôleur miroitait les limites selon le
  sens de marche, ce qui interdit d'apprendre les deux directions) ;
- **contrôle par couple PD** : la politique décale une pose de repos, un PD
  transforme l'écart en couple. L'ancien servo de vitesse (`SetMotorSpeed`)
  rendait le pantin instable en boucle fermée ;
- gravité 9.81 m/s² (le jeu utilise 18, trop violent pour l'apprentissage).

## Boucle d'entraînement

- réseau : 32 entrées → 32 → 32 → 11 sorties (tanh), 2475 paramètres ;
- observations : 11 angles, 11 vitesses articulaires, angle/vitesse du torse,
  vitesse du centre de masse, hauteur du torse, contact des deux pieds,
  horloge de phase, commande (−2/0/+2) ;
- actions : décalage de cible articulaire, borné par le PD ;
- récompense : suivi de vitesse (`cmd·vx`), hauteur du torse, verticalité,
  pénalités de dérive (commande nulle), d'énergie, de saccade et de chute ;
- optimisation : **OpenAI-ES** (48 échantillons antitétiques, rangs centrés,
  Adam, σ de 0.08 à 0.015). Pas de rétropropagation, donc aucune librairie ML :
  un MLP + ES en C pur suffit et évite une dépendance lourde/non packagée
  (ggml, libtorch C++). Brancher ggml/PPO reste possible si tu veux.
- curriculum : 100 générations équilibre, 100 à gauche, 100 à droite, puis
  mélange (50 % immobile, 25 % gauche, 25 % droite) ;
- checkpoint : `policy.bin` n'est écrit que si le score de **validation**
  (3 commandes, sans poussée) s'améliore. C'est ce que lit `eval`.

## Commandes

```sh
export PKG_CONFIG_PATH=/tmp/us-deps/prefix/lib/pkgconfig   # seulement si box2d est là
make -C ml
./ml/train 1500            # entraînement (~2 min)
./ml/train 1500 --resume   # affinage depuis policy.bin
./ml/eval                  # évalue policy.bin sur -1 / 0 / +1
./ml/probe                 # diagnostic de la pose neutre
make -C ml view && ./ml/view   # visualiseur Raylib (hors `all`) : A/D, R, Espace, molette
./ml/view autre.bin            # visualiseur avec un autre fichier de poids
```

## Visualiseur

`ml/view.c` dessine le même `Env` que `eval` (donc exactement ce qui est
évalué, aucune logique dupliquée) : booléens, géométrie Box2D et politique
partagés.

- **A/D** ou flèches : commande gauche/droite (relance une tentative à chaque
  changement, comme en évaluation) ; **R** : reset ; **Espace** : impulsion
  horizontale pour tester l'auto-équilibre ; **molette** : zoom.
- caméra qui suit en x avec lissage, cadrage fixe en y (sol toujours visible),
  zoom par défaut 240 px/m ; repère pointillé au point de départ et graduations
  au sol pour que le déplacement se voie malgré la caméra qui suit.
- HUD : commande, `dx`, vitesse, verticalité, hauteur du torse, contact des
  pieds, durée de l'épisode et « CHUTE après X s ».

Le binaire reste hors de `all` : `make -C ml` ne demande pas Raylib, seul
`make -C ml view` en a besoin.

`train` et `eval` écrivent/lisent `ml/policy.bin` (ignoré par git).

## Résultats mesurés (1500 générations, checkpoint validé)

| commande | déplacement sur 4 s | upright moyen | durée avant chute |
|---|---|---|---|
| gauche | −0.155 m | 0.91 | ~0.7 s |
| immobile | −0.479 m | 0.92 | ~0.95 s |
| droite | +0.093 m | 0.89 | ~1.1 s |

Les deux directions sont suivies, mais **l'équilibre n'est pas robuste** : la
politique tombe encore en une seconde et dérive quand on lui demande de rester
sur place. Le score de validation progresse (≈ +10 → +171) et les blocs
directionnels atteignent une récompense moyenne de 356 (gauche) et 76 (droite).

## Limites / suite

- la politique n'est **pas** branchée dans `src/ragdoll.c` : le jeu est intact ;
- l'équilibre demande plus d'échantillons (population/générations) ou un vrai
  algorithme de gradient (PPO), et un curriculum plus long sur l'immobilité ;
- prochaines étapes : visualiseur Raylib du ragdoll Old piloté par la politique
  (caméra qui suit et cadre le pantin), puis intégration derrière un flag.
