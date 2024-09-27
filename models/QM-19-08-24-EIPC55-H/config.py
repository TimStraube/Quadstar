## Parameter
orient = "NED"
usePrecession = bool(False)
Ts = 0.001
Tf = 20
Ti = 0

## Quadlive
port = 5001

## Quadtest
testschritte = 1
nutze_pid = True

## Quadtrain

# Ordner des Modells
ordner = "190824/modelPi"
# Modell fürs Training laden
model_laden = False
# Episodenzahl Training
episoden = 1e9
# Maximale Länge einer Episode
episodenlänge = 1000
# Rendern von Trainingsepisoden
render_modell = False
selbstorchestriert = False
selbstorchestriert_interval = [15, 31]
start_position = [0, 0, 0]
sollposition = [0, 0, 0]

# NN
pi = [4, 8]
vf = [4, 8]

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
schritte_bis_update = 5
aktion_range_end2end = 200
aktion_mittelwert_end2end = 480
render_fortschrittsanzeige = True
belohnungsanteil_position = 0.0
belohnungsanteil_geschwindigkeit = 0.6
belohnungsanteil_drehlage = 0.4
belohnungsanteil_drehgeschwindigkeit = 0.0
belohnungsanteil_rotordifferenz = 0.0

## PID

aktion_range_pid = 9.9
aktion_mittelwert_pid = 10
random_pid = False
aktionen = 25

## Liveevaluation des Trainings
n_eval_episodes = 1000
evaluationsfrequenz = 100000

# Quadserial
doclear = False
doprint = True