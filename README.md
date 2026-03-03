# Maqueen Line Follower — Rust Embarqué

Projet d'expérimentation embarquée temps réel en **Rust bare metal** sur un robot suiveur de ligne **DFRobot Maqueen** piloté par une **micro:bit v1.3B**.

L'objectif est purement technique : pousser au maximum les performances d'un Cortex-M0 contraint pour du suivi de ligne déterministe et réactif

---

## Matériel cible

### micro:bit v1.3B

| Caractéristique | Valeur |
|---|---|
| MCU | **Nordic nRF51822** |
| Architecture | **ARM Cortex-M0** |
| Fréquence CPU | 16 MHz |
| RAM | **16 KB** |
| Flash | 256 KB |
| FPU | ❌ Absent |
| Interface de flash | DAPLink (USB Mass Storage + CMSIS-DAP) |

Le Cortex-M0 est l'architecture ARM la plus contrainte de la famille : pas de FPU, pas de division hardware, jeu d'instructions Thumb-2 réduit. Tout calcul flottant est émulé par software — ce qui rend l'optimisation d'autant plus intéressante.

### DFRobot Maqueen

Châssis robot éducatif conçu pour la micro:bit, embarquant :
- 2 moteurs DC avec pont en H I2C
- 2 capteurs infrarouges de suivi de ligne (GPIO)
- LEDs RGB
- Capteur ultrasonique (optionnel)

---

## Stack technique

| Composant | Choix |
|---|---|
| Langage | Rust `no_std` / `no_main` |
| Cible de compilation | `thumbv6m-none-eabi` |
| HAL | `microbit 0.16` + `embedded-hal 1.0` |
| Runtime | `cortex-m-rt` |
| Panic handler | `panic-halt` |
| Flash toolchain | `probe-rs` via CMSIS-DAP |

---

## Installation de l'environnement

### 1. Dépendances système

Debian :
```bash
sudo apt install binutils-arm-none-eabi
```
Arch :
```bash
sudo pacman -S arm-none-eabi-binutils 
```

### 2. probe-rs

```bash
curl --proto '=https' --tlsv1.2 -LsSf \
  https://github.com/probe-rs/probe-rs/releases/latest/download/probe-rs-tools-installer.sh | sh

source ~/.cargo/env
```

### 3. Cible Rust embarquée

```bash
rustup target add thumbv6m-none-eabi
```


---

## Compiler et flasher

```bash
# Compilation release
cargo build --release

# Flash automatique via probe-rs (micro:bit branchée en USB)
cargo run --release
```

La config `.cargo/config.toml` pointe automatiquement sur `nRF51822_xxAA` — pas besoin de préciser le chip manuellement.

---

## Configuration éditeur (VSCode / rust-analyzer)

`.vscode/settings.json` :

```json
{
    "rust-analyzer.cargo.target": "thumbv6m-none-eabi",
    "rust-analyzer.check.allTargets": false
}
```

---

## Références

- [micro:bit V1 Hardware](https://tech.microbit.org/hardware/1-3-revision/)
- [Nordic nRF51822 Product Spec](https://www.nordicsemi.com/Products/nRF51822)
- [microbit HAL crate (docs.rs)](https://docs.rs/microbit/latest/microbit/)
- [probe-rs](https://probe.rs/)
- [The Embedded Rust Book](https://docs.rust-embedded.org/book/)
- [RTIC — Real-Time Interrupt-driven Concurrency](https://rtic.rs/)