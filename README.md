# Quadstar

### Ziele

1. Einen Quadcopter entwickeln
2. Eine Regelung basierend auf Reinforcement Learning Algorithmen entwickeln
3. Eine Bachelorarbeit rund um die Entwicklung schreiben

### Config

Alle relevanten Parameter sind in dieser Datei zusammengefasst und können variiert werden.
Bestimmte Parametervariationen können zu Fehlern führen.

### Plot von einer Flugepisode

Die Testbench kann verwendet werden um Plots von einer Positions- und Lagetrajektorie des Quadcopters zu erstellen

```
python3 quadtest.py
```

### Quadcoptervisualisierung mit Flask

```
python3 quadlive.py 
```

### Training von Modellen mit Stable Baselines3

Die config.py Datei wird beim Training automatisch in den in der config.py angegebenen Trainingsordner kopiert.
Soll ein Modell mit quadtest.py oder quadlive.py visualisert werden kann die config.py Datei einfach manuel aus dem Trainingsordner in das Hauptverzeichnis kopiert werden und ist dann aktiv.

```
python3 quadtrain.py
```

### Joystick

```
python3 joystick.py
```
