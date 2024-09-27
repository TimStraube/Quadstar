## Parameter
orient = "NED"
usePrecession = bool(False)
Ts = 0.0005
Tf = 20
Ti = 0

## Quadlive
port = 5001

## Quadtest
testschritte = 1000
nutze_pid = False

## Quadtrain

# Ordner des Modells
ordner = "150824/modelEisenzeit"
# Modell fürs Training laden
model_laden = False
# Episodenzahl Training
episoden = 1e3
# Maximale Länge einer Episode
episodenlänge = 1000
# Rendern von Trainingsepisoden
render_modell = False
selbstorchestriert = True
selbstorchestriert_interval = [115, 120]
start_position = [0, 0, 0]
sollposition = [0, 0, 0]
render_fortschrittsanzeige = True
belohnungsanteil_position = 0.0
belohnungsanteil_geschwindigkeit = 0.2
belohnungsanteil_drehlage = 0.8
belohnungsanteil_drehgeschwindigkeit = 0.0
belohnungsanteil_rotorvarianz = 0.0

## Reglertyp
# "model", "pid"
reglertyp = "pid"

## Model

# Lernrate
lernrate = 0.00001
# Parallel Umwelten fürs Training
n_envs = 4
# Batchgröße
batchmenge = 16

## Model
# Schritte bis das Modell ein neue Parameter lernt
schritte_bis_update = 10
aktion_range_end2end = 200
aktion_mittelwert_end2end = 480

## PID

aktion_range_pid = 9.9
aktion_mittelwert_pid = 10
random_pid = False
aktionen = 25

## Liveevaluation des Trainings
n_eval_episodes = 100
evaluationsfrequenz = 1000

# Quadserial
doclear = False
doprint = True