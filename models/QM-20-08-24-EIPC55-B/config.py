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
ordner = "200824/modelSelbstLWord"
# Modell fürs Training laden
model_laden = False
# Episodenzahl Training
episoden = 1e9
# Maximale Länge einer Episode
episodenlänge = 10000
# Rendern von Trainingsepisoden
render_modell = False
selbstorchestriert = True
selbstorchestriert_interval = [4, 5]
start_position = [0, 0, 0]
sollposition = [0, 0, 0]

# NN
pi = [2, 2]
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
# Schritte bis das Modell ein neue Parameter lernt
schritte_bis_update = 1
aktion_range_end2end = 200
aktion_mittelwert_end2end = 480
render_fortschrittsanzeige = True
belohnungsanteil_position = 0.0
belohnungsanteil_geschwindigkeit = 0.2
belohnungsanteil_drehlage = 0.6
belohnungsanteil_drehgeschwindigkeit = 0.2
belohnungsanteil_rotordifferenz = 0.0

## PID

aktion_range_pid = 9.9
aktion_mittelwert_pid = 10
random_pid = False
aktionen = 15

## Liveevaluation des Trainings
n_eval_episodes = 1000
evaluationsfrequenz = 100000

# Quadserial
doclear = False
doprint = True