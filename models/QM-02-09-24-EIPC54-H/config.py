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
nutze_pid = False

## Quadtrain

# Ordner des Modells und Reglertypdefinition
# [0] = Quad
# [1] = {M (Ende-zu-Ende) oder P (PID)}

ordner = "QM-02-09-24-EIPC54-H"
# Modell fürs Training laden
model_laden = False
# Episodenzahl Training
episoden = 1e9
# Maximale Länge einer Episode
episodenlänge = 1000
# Rendern von Trainingsepisoden
render_modell = False
aktionsraum = [1000, 1000, 1000, 1000, 10]
start_position = [0, 0, 0]
sollposition = [0, 0, 0]

# NN
pi = [4, 4]
vf = [4, 4]

## Model

# Lernrate
lernrate = 0.00000001
# Parallel Umwelten fürs Training
n_envs = 1
# Batchgröße
batchmenge = 16

## Model
# Schritte bis das Modell einen neuen Parameter lernt
render_fortschrittsanzeige = True
belohnungsanteil_position = 0.0
belohnungsanteil_geschwindigkeit = 0.9
belohnungsanteil_drehlage = 0.1
belohnungsanteil_drehgeschwindigkeit = 0.0
belohnungsanteil_rotordifferenz = 0.0

## PID

aktion_range_pid = 4.99999
aktion_mittelwert_pid = 10
random_pid = False
aktionen = 18

## Liveevaluation des Trainings
n_eval_episodes = 2
evaluationsfrequenz = 10000

# Quadserial
doclear = False
doprint = True
joystick2serial = True