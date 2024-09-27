## Parameter
orient = "NED"
usePrecession = bool(False)
Ts = 0.001
Tf = 20
Ti = 0

## Quadlive
port = 5001
liveschrittweite = 1

## Quadtest
testschritte = 1000
nutze_pid = True

## Quadtrain

# Ordner des Modells
ordner = "QM-28-08-24-EIPC55-B"
# Modell fürs Training laden
model_laden = False
# Episodenzahl Training
episoden = 1e9
# Maximale Länge einer Episode
episodenlänge = 10000
# Rendern von Trainingsepisoden
render_modell = False
aktionsraum = [10, 10, 10, 10, 1]
start_position = [0, 0, 0]
sollposition = [0, 0, 0]

# NN
pi = [64, 64]
vf = [64, 64]

## Model

# Lernrate
lernrate = 0.0001
# Parallel Umwelten fürs Training
n_envs = 1
# Batchgröße
batchmenge = 16

## Model
# Schritte bis das Modell einen neuen Parameter lernt
render_fortschrittsanzeige = True
belohnungsanteil_position = 0.0
belohnungsanteil_geschwindigkeit = 1.0
belohnungsanteil_drehlage = 0.0
belohnungsanteil_drehgeschwindigkeit = 0.0
belohnungsanteil_rotordifferenz = 0.0

## PID

aktion_range_pid = 4.9
aktion_mittelwert_pid = 5
random_pid = False
aktionen = 18

## Liveevaluation des Trainings
n_eval_episodes = 2
evaluationsfrequenz = 10000

# Quadserial
doclear = False
doprint = True