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
testschritte = 1
nutze_pid = True

## Quadtrain

# Ordner des Modells
ordner = "220824/modelAsien"
# Modell fürs Training laden
model_laden = False
# Episodenzahl Training
episoden = 1e9
# Maximale Länge einer Episode
episodenlänge = 3000
# Rendern von Trainingsepisoden
render_modell = False
aktionsraum = [10, 10, 10, 10, 20]
start_position = [0, 0, 0]
sollposition = [0, 0, 0]

# NN
pi = [8, 8]
vf = [8, 8]

## Reglertyp
# "model", "pid"
reglertyp = "model"

## Model

# Lernrate
lernrate = 0.000001
# Parallel Umwelten fürs Training
n_envs = 4
# Batchgröße
batchmenge = 16

## Model
# Schritte bis das Modell einen neuen Parameter lernt
schritte_bis_update = 1
aktion_range_end2end = 100
aktion_mittelwert_end2end = 480
render_fortschrittsanzeige = True
belohnungsanteil_position = 0.0
belohnungsanteil_geschwindigkeit = 0.4
belohnungsanteil_drehlage = 0.2
belohnungsanteil_drehgeschwindigkeit = 0.2
belohnungsanteil_rotordifferenz = 0.2

## PID

aktion_range_pid = 24.999
aktion_mittelwert_pid = 25
random_pid = False
aktionen = 25

## Liveevaluation des Trainings
n_eval_episodes = 2
evaluationsfrequenz = 10000

# Quadserial
doclear = False
doprint = True