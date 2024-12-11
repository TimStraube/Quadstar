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
nutze_pid = False

## Quadtrain

# Ordner des Modells
ordner = "280824/pidE"
# Modell fürs Training laden
model_laden = False
# Episodenzahl Training
episoden = 1e9
# Maximale Länge einer Episode
episodenlänge = 500
# Rendern von Trainingsepisoden
render_modell = False
aktionsraum = [10, 10, 10, 10, 1]
start_position = [0, 0, 0]
sollposition = [0, 0, 0]

# NN
pi = [8, 8]
vf = [8, 8]

## Reglertyp
# "model", "pid"
reglertyp = "pid"

## Model

# Lernrate
lernrate = 0.01
# Parallel Umwelten fürs Training
n_envs = 1
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
belohnungsanteil_drehlage = 0.3
belohnungsanteil_drehgeschwindigkeit = 0.3
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