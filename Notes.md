# Positionnement des LEDs Micro::bit v1.3B

Les LEDs forment une matrice 5x5 mais se contrôlent via une matrice 3x9.
Voici la correspondance :

R = Row (est la ligne de la matrice 3x9)  
C = Column (est la colonne de la matrice 3x9)

R2-C8 et R2-C9 ne sont pas utilisés.

| | | | | |
|:---:|:---:|:---:|:---:|:---:|
| **R1** C1 | **R2** C4 | **R1** C2 | **R2** C5 | **R1** C3 |
| **R3** C4 | **R3** C5 | **R3** C6 | **R3** C7 | **R3** C8 |
| **R2** C2 | **R1** C9 | **R2** C3 | **R3** C9 | **R2** C7 |
| **R1** C8 | **R1** C7 | **R1** C6 | **R1** C5 | **R1** C4 |
| **R3** C3 | **R2** C7 | **R3** C1 | **R2** C6 | **R3** C2 |


# Annexe Technique : Driver Robot Maqueen (ROB0148) pour micro:bit

## 1. Configuration Matérielle Globale
*   **Plateforme Cible :** BBC micro:bit V1.3b (nRF51822).
*   **Interface Principale :** I2C (Inter-Integrated Circuit) pour les actuateurs.
*   **Interface Secondaire :** GPIO Direct pour les capteurs et LEDs.
*   **Fréquence I2C :** Standard (100kHz).

---

## 2. Cartographie des Broches (Pinout Map)

Ces broches doivent être configurées dans le `Board` struct (HAL).

| Composant | Pin micro:bit | Type I/O | Notes |
| :--- | :--- | :--- | :--- |
| **I2C SDA** | **P20** | I/O | Bus de commande Moteurs/Servos |
| **I2C SCL** | **P19** | Output | Bus de commande Moteurs/Servos |
| **Ultrason (Trig)** | **P1** | Output | Envoyer une impulsion de 10µs |
| **Ultrason (Echo)** | **P2** | Input | Mesurer la durée de l'impulsion retour |
| **LED Avant Gauche**| **P8** | Output | `1` = Allumé, `0` = Éteint |
| **LED Avant Droite**| **P12** | Output | `1` = Allumé, `0` = Éteint |
| **Capteur Ligne G** | **P13** | Input | `0` = Blanc/Réfléchissant, `1` = Noir/Vide |
| **Capteur Ligne D** | **P14** | Input | `0` = Blanc/Réfléchissant, `1` = Noir/Vide |
| **Récepteur IR** | **P16** | Input | Décodage protocole NEC |
| **Buzzer** | **P0** | Output | Génération PWM pour le son |

---

## 3. Protocole I2C (Moteurs & Servos)

Le micro-contrôleur du Maqueen agit comme un esclave I2C.

*   **Adresse I2C Esclave :** `0x10` (16 décimal)

### A. Contrôle des Moteurs
Pour piloter un moteur, écrire 3 octets consécutifs sur l'I2C : `[REGISTRE, DIRECTION, VITESSE]`

| Moteur | Registre | Direction (Val) | Vitesse (Val) |
| :--- | :--- | :--- | :--- |
| **Moteur Gauche (M1)** | `0x00` | `0` = Avant (CW)<br>`1` = Arrière (CCW) | `0` à `255` |
| **Moteur Droit (M2)** | `0x02` | `0` = Avant (CW)<br>`1` = Arrière (CCW) | `0` à `255` |

*Exemple de trame I2C pour Moteur Gauche, Avant, Vitesse 100 :*
`write(0x10, [0x00, 0x00, 0x64])`

### B. Contrôle des Servomoteurs
Pour piloter un servo, écrire 2 octets : `[REGISTRE, ANGLE]`

| Servo | Registre | Angle (Val) |
| :--- | :--- | :--- |
| **Servo S1** | `0x14` (20 décimal) | `0` à `180` (degrés) |
| **Servo S2** | `0x15` (21 décimal) | `0` à `180` (degrés) |

### C. Informations Système
| Info | Registre | Lecture |
| :--- | :--- | :--- |
| **Version Firmware** | `0x32` (50 décimal) | Lire les octets disponibles |

---

## 4. Logique des Capteurs (Détails d'implémentation)

### Capteur Ultrason (HC-SR04)
Le Maqueen adapte le niveau logique (5V vers 3.3V).
1.  Mettre **P1** à `High` pendant >10µs, puis `Low`.
2.  Attendre que **P2** passe à `High`.
3.  Mesurer le temps que **P2** reste à `High` (Pulse In).
4.  **Formule :** `Distance (cm) = Durée (µs) / 58`.

### Capteurs de Ligne (Suiveur)
*   **P13 (Gauche) / P14 (Droit)**
    *   `Low (0)` : Le capteur voit une surface réfléchissante (BLANC).
    *   `High (1)` : Le capteur voit une surface absorbante (NOIR) ou le vide.

### Récepteur IR
*   **Pin P16**
*   Protocole : **NEC**.
*   Timing header : 9ms High + 4.5ms Low.
*   Logique bit : Mesure de la distance entre les fronts descendants.
