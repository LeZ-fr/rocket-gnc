# Protocole TCP Python <-> flight_sw (v1, brouillon)

Mode lockstep : Python envoie une mesure, attend l'estimation, puis avance d'un pas.
Little-endian, messages de taille fixe, sans padding. `TCP_NODELAY` des deux côtés.

## Python -> C : mesure

| Champ | Type |
|---|---|
| timestamp (µs) | uint64 |
| accélération (m/s²) | float32 |
| pression baro (Pa) | float32 |
| baro valide | uint8 |

## C -> Python : estimation

| Champ | Type |
|---|---|
| timestamp (µs) | uint64 |
| altitude estimée (m) | float32 |
| vitesse estimée (m/s) | float32 |
| variance altitude | float32 |
| variance vitesse | float32 |
