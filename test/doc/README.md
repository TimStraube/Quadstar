# Test

Dieser Ordner umfasst alle Unit- und Integrationstests.

## Unittests

### Übersicht

Die wichtigsten Komponenten welche mit Unittests versehen sind oder versehen werden sollen sind:

- Die ESCs (Electronic speed controllers)
- Die MCU (Main Compute Unit)
- Die CUs (Communication Units)
- Quadui

## Integrationstests

### Übersicht

Die folgenden Systeme sollen mit Integrationstests getestet werden

- Machanisches System
- Elektrisches System
- Software System
- Gesamtintegration vor Testflügen

Das Mechanische, Elektrische und Software System können unabhängig voneinander getestet werden vorausgesetzt alle für einen Test erforderlichen Komponenten sind getestet. Bestimmte Softwaretests sind beispielsweise nur sinnvoll durchführbar wenn bestimmte elektrische oder mechanische Komponenten vorhanden und getestet sind. Im Laufe der Zeit soll eine Abhängigkeitsdiagramm entwickelt werden welche Abhängigkeiten und Testabläufe strukturiert und Testpfade aufzeigt.

## Testziele

Für Tests gelten die Anforderungen:

- Ein Test kann nur als Bestanden gewertet wenn alle Test einer Testreihe mit mindestens fünf Epochen Bestanden bewertet wurden.
- Testflüge dürfen nur durchgeführt werden wenn der Gesamtintegrationstest bestanden wurde.