# Motor ESC Test

## Setup

### Beschreibung

Test of a single Motor ESC circuit.

| **Testbeschreibung** | Wert           |
| -------------------- | -------------- |
| ID                   | 0001           |
| Typ                  | Motor ESC Test |
| Datum                | 21.09.2025     |
| Startzeit            | 19:30          |
| Endzeit              | 21:30          |
| Testername           | Tim Straube    |

### Hardware

| **Hardware**                          | Wert            |
| ------------------------------------- | --------------- |
| Typ ESC                               | STM BG431B ESC1 |
| ID ESC                                | BG431B_ESC1_1   |
| Typ Motor                             |                 |
| Anzahl ESCs                           | 1               |
| Quellenseitige Verschaltungstopologie | Seriell         |
| Eingangsspannung                      | 20V             |
| Eingangsstrom                         |                 |

| **Software**  | Wert  |
| ------------- | ----- |
| Version (Tag) | 1.0.0 |

## Ergebnis

### Erfolgsanforderungen

| **Erfolgsanforderungen**                 | Resultat    |
| ---------------------------------------- | ----------- |
| Firmware Upload                          |             |
| Regelung mit PWM                         | Erfolgreich |
| Beide grünen LEDs leuchten beim power up |             |
| Rote LED leuchtet beim power up          |             |
| --                                       | --          |
| Gesamtergebnis                           | Failed      |

### Verhaltensbeschreibung

...
