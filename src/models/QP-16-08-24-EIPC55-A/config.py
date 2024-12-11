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
ordner = "160824/pidSaturn"
# Modell fürs Training laden
model_laden = False
# Episodenzahl Training
episoden = 1e7
# Maximale Länge einer Episode
episodenlänge = 100
# Rendern von Trainingsepisoden
render_modell = False
selbstorchestriert = False
selbstorchestriert_interval = [15, 31]
start_position = [0, 0, 0]
sollposition = [0, 0, 0]

## Reglertyp
# "model", "pid"
reglertyp = "pid"

## Model

# Lernrate
lernrate = 1e-4
# Parallel Umwelten fürs Training
n_envs = 4
# Batchgröße
batchmenge = 16

## Model
# Schritte bis das Modell ein neue Parameter lernt
schritte_bis_update = 1
aktion_range_end2end = 400
aktion_mittelwert_end2end = 480
render_fortschrittsanzeige = True
belohnungsanteil_position = 0.0
belohnungsanteil_geschwindigkeit = 0.0
belohnungsanteil_drehlage = 0.1
belohnungsanteil_drehgeschwindigkeit = 0.8
belohnungsanteil_rotordifferenz = 0.1

## PID

aktion_range_pid = 9.9
aktion_mittelwert_pid = 10
random_pid = False
aktionen = 25

## Liveevaluation des Trainings
n_eval_episodes = 100
evaluationsfrequenz = 10000

# Quadserial
doclear = False
doprint = True