# rocket-gnc

Projet de chaîne GNC (Guidance, Navigation, Control) / FDIR pour une fusée, entièrement simulé sur ordinateur.

## Architecture

```
Simulateur (vérité) -> Modèles de capteurs -> flight_sw (C) -> Estimation
     (Python)              (Python)           via TCP         -> Visualisation
```

- `sim/` : physique (vecteurs 3D, mode 1D/2D/3D par contraintes), modèles de capteurs
- `viz/` : animation (planète, fusée, traînées, sélecteur 1D/2D/3D)
- `flight_sw/` : logiciel de vol en C (filtre de Kalman 1D, puis EKF) -- à venir
- `docs/` : documentation, dont le protocole TCP

Règle d'or : `flight_sw` ne lit jamais la vérité terrain, uniquement les mesures capteurs.

## Feuille de route

1. [ ] Physique vectorielle + planète + fusée (points, traînées)
2. [ ] Sélecteur 1D / 2D / 3D
3. [ ] Modèles de capteurs (accéléromètre, baromètre)
4. [ ] `flight_sw` en C : Kalman 1D
5. [ ] Liaison TCP en lockstep (voir `docs/PROTOCOL.md`)
6. [ ] Affichage vérité vs mesures vs estimation
7. [ ] 3D : gyroscope, EKF attitude, FDIR, contrôle

## Installation

```bash
python -m venv .venv
source .venv/bin/activate    # Windows : .venv\Scripts\activate
pip install -r requirements.txt
```
