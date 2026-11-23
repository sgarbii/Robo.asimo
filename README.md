# Robo.asimo

Autonomous obstacle-avoiding robot built on an Arduino Uno. An ultrasonic sensor mounted on a servo scans left, front and right, and a state machine decides where to go.

## Hardware

| Part | Role |
|---|---|
| Arduino Uno | Controller |
| L298N | Motor driver for the two rear DC motors (2WD chassis, caster wheel in front) |
| HC-SR04 | Ultrasonic distance sensor |
| SG90 servo | Rotates the sensor to scan directions |

### Wiring

| Signal | Arduino pin |
|---|---|
| ENA (left motor PWM) | 5 |
| IN1 / IN2 | 7 / 8 |
| ENB (right motor PWM) | 6 |
| IN3 / IN4 | 11 / 12 |
| Servo | 9 |
| HC-SR04 TRIG / ECHO | A0 / A1 |

ENA and ENB avoid pins 9 and 10 because the Servo library uses Timer1, which disables PWM on those pins.

## How it works

The robot runs a state machine with five states: **Advance → Stop → Scan → Turn → Confirm**.

- **Advance:** drives forward while the sensor sweeps 60°, 90° and 120°. It slows down when something is closer than 60 cm and stops below 35 cm (22 cm for the side readings).
- **Scan:** checks left, front and right, and picks the first direction with at least 50 cm of free space.
- **Turn and Confirm:** turns toward the chosen direction, then checks again. If the path still isn't clear, it corrects in 30° steps (up to 4 times) before scanning again.
- **Stuck detection:** if the front reading doesn't change for 3 s (8 s with no echo), the robot assumes it is stuck, backs up and looks for another way.

Each distance is the median of 3 valid readings, which filters out noise and false echoes.

## Safety limits

The motors are rated for 6 V but receive about 10 V, so PWM is capped at 170. A `static_assert` stops the build if any speed constant goes above that limit.

## Test modes

Set one flag to `true` at the top of `src/main.cpp` (only one at a time):

| Flag | What it does |
|---|---|
| `MODO_TESTE_SENSOR` | Prints distances with the motors off |
| `MODO_TESTE_SERVO` | Moves the servo left, front and right, motors off |
| `MODO_CALIBRACAO` | Turns 90° and pauses, to tune `MS_POR_GRAU` on the real floor |

`DEBUG` prints state changes, scan results and the chosen angle to the serial monitor (9600 baud).

## Build and upload

Requires [PlatformIO](https://platformio.org/) (VS Code extension or CLI).

```bash
pio run --target upload
pio device monitor
```

## Tuning

- `VEL_ESQ_TRIM` / `VEL_DIR_TRIM`: compensate if the robot drifts to one side.
- `INVERTER_ESQ` / `INVERTER_DIR`: flip a wheel that spins the wrong way.
- `PULSO_DIREITA`, `PULSO_FRENTE`, `PULSO_ESQUERDA`: adjust the servo positions.
