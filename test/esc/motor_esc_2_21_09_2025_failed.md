# Motor ESC Test

## Setup

### Beschreibung

Test of a single Motor ESC circuit.

| **Testbeschreibung** | Wert           |
| -------------------- | -------------- |
| ID                   | 0002           |
| Typ                  | Motor ESC Test |
| Datum                | 21.09.2025     |
| Startzeit            | 21:25          |
| Endzeit              | 21:30          |
| Testername           | Tim Straube    |

### Hardware

| **Hardware**                          | Wert             |
| ------------------------------------- | ---------------- |
| Typ ESC                               | STM BG431B ESC1  |
| ID ESC                                | BG431B_ESC1_Q3_4 |
| Typ Motor                             |                  |
| Anzahl ESCs                           | 1                |
| Quellenseitige Verschaltungstopologie | Seriell          |
| Eingangsspannung in V                 | 20               |
| Maximaler Eingangsstrom in A          | 3.5              |

| **Software**  | Wert  |
| ------------- | ----- |
| Version (Tag) | 1.0.1 |

## Ergebnis

### Erfolgsanforderungen

| **Erfolgsanforderungen**                                  | Resultat (failed 0, success 1) |
| --------------------------------------------------------- | ------------------------------ |
| Firmware Upload                                           | 1                              |
| Regelung mit PWM                                          | 0                              |
| Beide grünen LEDs leuchten beim power up                  | 1                              |
| Rote LED leuchtet beim power up                           | 1                              |
| Drehzahlverhalten entspricht den Erwartungen              | 0                              |
| 3 Testzyklen mit identischem Verhalten bei gleichem Input | 0                              |
| ESC hatte keine Übertemperatur                            | 0                              |
| Motor hatte keine Übertemperatur                          | 0                              |
| Voll Schubkraft erreicht                                  | 0                              |
| Sound                                                     | 0                              |

### Verhaltensbeschreibung

Der Motor brummt laut. Vermutlich beim Sprung von Sollgeschwindigkeit 0 auf 50 bewegt er sich zwar kurz verfängt sich dann aber in einem oszilierendem Zustand mit nur kleinen Winkeländerungen aber keiner vollen Drehung.
