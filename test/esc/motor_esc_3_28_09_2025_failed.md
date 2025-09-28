# Motor ESC Test

## Setup

### Beschreibung

Test of a single Motor ESC circuit.

| **Testbeschreibung** | Wert           |
| -------------------- | -------------- |
| ID                   | 0003           |
| Typ                  | Motor ESC Test |
| Datum                | 28.09.2025     |
| Startzeit            | 22:30          |
| Endzeit              | 23:30          |
| Testername           | Tim Straube    |

### Hardware

| **Hardware**                           | Wert            |
| -------------------------------------- | --------------- |
| Typ ESC                                | STM BG431B ESC1 |
| ID ESC                                 | BG431B_ESC1_1   |
| Typ Motor                              |                 |
| Anzahl ESCs                            | 1               |
| Quellenseitige Verschaltungstopologie  | Seriell         |
| Eingangsspannung in V                  | 20              |
| Maximaler Eingangsstrom in A           |                 |
| Frequenz PWM Input                     | 420Hz           |
| PWM minimaler Input                    | 1400ms          |
| PWM maximaler Input                    | 1800ms          |
| PWM Steigung (Delta Input pro Delta t) | 1 / 10          |

| **Software**  | Wert  |
| ------------- | ----- |
| Version (Tag) | 1.0.0 |

## Ergebnis

### Erfolgsanforderungen

| **Erfolgsanforderungen**                                  | Resultat (failed 0, success 1) |
| --------------------------------------------------------- | ------------------------------ |
| Firmware Upload                                           | 1                              |
| Regelung mit PWM                                          | 0                              |
| Beide grünen LEDs leuchten beim power up                  | 0                              |
| Rote LED leuchtet beim power up                           | 1                              |
| Drehzahlverhalten entspricht den Erwartungen              | 0                              |
| 3 Testzyklen mit identischem Verhalten bei gleichem Input | 1                              |
| ESC hatte keine Übertemperatur                            | 1                              |
| Motor hatte keine Übertemperatur                          | 1                              |
| Voll Schubkraft erreicht                                  | 0                              |
| Sound                                                     | 0                              |

### Verhaltensbeschreibung

Wie erwartet dreht der Motor hoch auf 150. Doch sobald PWM greift beginnt er laut auf der Stelle zu ozillieren. 
Wahrscheinlich kommt das PWM Signal nicht an oder wird nicht richtig erkannt.