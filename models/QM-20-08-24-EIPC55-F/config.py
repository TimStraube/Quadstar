## Parameter
orient = "NED"
usePrecession = bool(False)
Ts = 0.0001
Tf = 20
Ti = 0

## Quadlive
port = 5001

## Quadtest
testschritte = 1000
nutze_pid = False

## Quadtrain

# Ordner des Modells
ordner = "200824/modelSelbstBit"
# Modell fürs Training laden
model_laden = False
# Episodenzahl Training
episoden = 1e9
# Maximale Länge einer Episode
episodenlänge = 20000
# Rendern von Trainingsepisoden
render_modell = False
selbstorchestriert = True
selbstorchestriert_interval = [49, 50]
start_position = [0, 0, 0]
sollposition = [0, 0, 0]

# NN
pi = [2, 2]
vf = [16, 16]

## Reglertyp
# "model", "pid"
reglertyp = "model"

## Model

# Lernrate
lernrate = 0.0001
# Parallel Umwelten fürs Training
n_envs = 1
# Batchgröße
batchmenge = 16

## Model
# Schritte bis das Modell ein neue Parameter lernt
schritte_bis_update = 1
aktion_range_end2end = 200
aktion_mittelwert_end2end = 480
render_fortschrittsanzeige = True
belohnungsanteil_position = 0.0
belohnungsanteil_geschwindigkeit = 0.0
belohnungsanteil_drehlage = 1.0
belohnungsanteil_drehgeschwindigkeit = 0.0
belohnungsanteil_rotordifferenz = 0.0

## PID

aktion_range_pid = 9.9
aktion_mittelwert_pid = 10
random_pid = False
aktionen = 25

## Liveevaluation des Trainings
liveschrittweite = 1
n_eval_episodes = 100
evaluationsfrequenz = 10000

# Quadserial
doclear = False
doprint = True