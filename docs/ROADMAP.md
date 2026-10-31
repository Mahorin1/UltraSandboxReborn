# Roadmap : une tâche = un commit. Cocher quand fait.

## Phase 0 : base (fait)
- [x] Makefile, boucle pas fixe, Box2D v3 (workers internes), Raylib
- [x] Port du ragdoll (tailles, joints, limites, IK jambes, équilibre, capteurs de sol)

## Phase 1 : outillage
- [x] Overlay debug (F1) : joints, capteurs, état du ragdoll
- [x] CI GitHub Actions : build Box2D + `make test`
- [x] Test headless avec assertions (ragdoll debout après 10 s)

## Phase 2 : migration depuis UltraSandbox_Old
- [x] 2.1 Drag souris sur les corps (motor joint ressort, DragJoint) + rotation Q/E
- [x] Active ragdoll : équilibre, marche, course, accroupissement, relevé par appui des coudes
- [ ] 2.2 Entités spawnables : caisse (dégradé), tonneau, scie (textures, son de choc)
- [ ] 2.3 Ressources : gestionnaire (textures/sons) et audio (limite de sons simultanés, pitch)
- [ ] 2.4 Caméra : drag, zoom avec tween, shake, suivi
- [ ] 2.5 Slow-motion avec tween (S), pitch des sons
- [ ] 2.6 Cordes (StaticRope catenary, DynamicRope), splines
- [ ] 2.7 GUI : panneau latéral d'outils, boutons animés, menu principal
- [ ] 2.8 Robot (roue + équilibre PID) et Guirlande

## Phase 3 : architecture
- [ ] ECS minimal (sparse-set) : entités, composants, systèmes, ressources
- [ ] Migration progressive : ragdoll et entités en composants
- [ ] Planificateur de systèmes ; parallélisation après profilage (Tracy)

## Phase 4 : plateforme (univers/OS)
- [ ] Scènes, sauvegarde, éditeur, modularité
