# AGENTS.md : règles pour les agents (Copilot CLI/agent, Claude)

## Projet
Ultra Sandbox Reborn : bac à sable physique 2D, active ragdoll. C17, Raylib, Box2D 3.2. Linux (CachyOS/Hyprland).
Anciennes sources de référence : repo UltraSandbox_Old (C++/SFML2/Box2D v2). Ne pas copier du C++ : porter en C.

## Commandes
- `make` / `make run` : debug (ASan+UBSan), `make release` : -O3
- `make test` : simulation headless (Box2D seul), doit passer avant tout commit
- Après modification de fichiers : toujours compiler et lancer `make test`.

## Structure
src/util (maths, PID) · src/physics (monde, pas fixe 1/80, workers) · src/ragdoll · src/render (Raylib uniquement ici et main.c)
`ragdoll.c`, `physics.c`, `util.c` ne doivent PAS dépendre de Raylib (le test headless les compile seuls).

## Conventions
- C17 pur, pas de C++. Pas de variables globales mutables hors d'un struct passé en paramètre.
- Unités : pixels dans les données de jeu, mètres dans Box2D (conversion `PPM`=64 via px2m/m2px). Axe y vers le bas, gravité (0, 18).
- Data-oriented : structs plats, tableaux contigus, indices/handles plutôt que pointeurs ; aucune allocation dans la boucle chaude.
- Box2D v3 : API par handles (`b2BodyId`...), jamais de pointeur vers un body. Limites de revolute bornées à ±180°.
- Pas de dépendance ajoutée sans l'accord de l'utilisateur ; préférer les paquets système (pacman).
- Commits petits, un sujet par commit, message en anglais impératif.
- Aucun secret/token dans le repo.

## Style de réponse
Concis, technique. Montrer le diff plutôt que le fichier entier.
