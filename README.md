# Balise DGAC (Remote ID) — ESP32-C3

Firmware ESP32-C3 qui émet une balise DGAC (trame WiFi beacon 802.11, canal 6) à partir des données GPS d'un contrôleur de vol Betaflight lu en MSP.

- Position, altitude, vitesse, cap et point home (pris à l'armement)
- Émission toutes les 3 s ou tous les 30 m parcourus, uniquement après un fix GPS
- LED de statut (GPIO8) : fixe quand la balise émet

## Configuration
Dans `balise_dgac/balise_dgac.ino` :
- `ID_FR` : **remplace l'identifiant bidon par ton propre identifiant DGAC** (30 caractères exactement).
- MSP : RX = GPIO20, TX = GPIO21, 115200 baud.

## Compilation
Arduino IDE ou `arduino-cli`, carte `esp32:esp32:esp32c3`.
