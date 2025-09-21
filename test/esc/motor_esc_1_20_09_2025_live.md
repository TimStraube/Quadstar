# Motor ESC Test

## Setup

### Beschreibung

Test of a single Motor ESC circuit.

| **Testbeschreibung** | Wert           |
| -------------------- | -------------- |
| ID                   | 0001           |
| Typ                  | Motor ESC Test |
| Datum                | 21.09.2025     |
| Startzeit            | 20:00          |
| Endzeit              |                |
| Testername           | Tim Straube    |

### Hardware

| **Hardware**                          | Wert             |
| ------------------------------------- | ---------------- |
| Typ ESC                               | STM BG431B ESC1  |
| ID ESC                                | BG431B_ESC1_Q3_4 |
| Typ Motor                             |                  |
| Anzahl ESCs                           | 1                |
| Quellenseitige Verschaltungstopologie | Seriell          |
| Eingangsspannung                      | 20V              |
| Eingangsstrom                         |                  |

| **Software**  | Wert  |
| ------------- | ----- |
| Version (Tag) | 1.0.0 |

## Ergebnis

### Erfolgsanforderungen

| **Erfolgsanforderungen**                                  | Resultat (failed 0, success 1) |
| --------------------------------------------------------- | ------------------------------ |
| Firmware Upload                                           | 1                              |
| Regelung mit PWM                                          | 0                              |
| Beide grünen LEDs leuchten beim power up                  | 1                              |
| Rote LED leuchtet beim power up                           | 1                              |
| Drehzahlverhalten entspricht den Erwartungen              | 1                              |
| 3 Testzyklen mit identischem Verhalten bei gleichem Input | 1                              |
| ESC hatte keine Übertemperatur                            | 0                              |
| Motor hatte keine Übertemperatur                          | 0                              |
| Voll Schubkraft erreicht                                  | 0                              |
| Sound                                                     | 0                              |

### Verhaltensbeschreibung

Der Motor dreht dreimal hoch kurz nachdem die Spannung erhöht wird und spult wieder runter. Es kommt nicht zu einem Abbruch. Besonders der ESC ist nach den drei Testzyklen sehr warm.
