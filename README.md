# Ultra Sandbox Reborn
C17 · Raylib · Box2D 3.2 · (ECS + jobs à venir)

    sudo pacman -S raylib box2d base-devel   # ou via ton gestionnaire
    make run          # debug (ASan/UBSan)
    make release
    make test         # simulation headless du ragdoll (Box2D seul)

Workers physique : `US_WORKERS=N ./ultrasandbox` (défaut ≈ nproc/2).
Arborescence : `src/util` (maths, PID) · `src/physics` (monde, pas fixe) · `src/ragdoll` (port de l'ancien Ragdoll) · `src/render` (Raylib) · `src/main`.
