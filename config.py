## Parameter
# NED = [North, East, Down]
# [s]
step_interval = 0.001
# [s]
episode_end_time = 10
# [s]
episode_start_time = 0

## Quadtrain
# Foldername and id for the model
model_id = "QP-23-08-24-EIPC55-C"
# Load model for continued training
load_model = False
# Maximal number of episodes
episodes = 1e8
# Network architecture
actor = [2, 2]
critic = [2, 2]
# Gradient descent learning rate
learning_rate = 1e-4
# Parallel environments
parallel_environments = 4
# Batchsize
batchsize = 16

## Quadend2end
# [Motor 1, Motor 2, Motor 3, Motor 4, Updateperiode [s]]
Aktionsraum_EndeZuEnde = [1000, 1000, 1000, 1000, 1]
# Position, Geschwindigkeit, Drehlage, Drehgeschwindigkeit
Belohnungsgewichtung = [0.0, 0.4, 0.2, 0.2]

## Quadpid
Aktionsinterval_PID = [0, 10]
# Zahl der zulernenden PID-Parameter
Aktionen = 6
# Updateperiode [s]
Updateperiode_PID = 0.01
# Auszug der zulernenden PID-Parameter
# self.controller.vel_P_gain[0] = aktion[0]
# self.controller.vel_P_gain[1] = aktion[0]
# self.controller.vel_P_gain[2] = aktion[1]

# self.controller.vel_I_gain[0] = aktion[2]
# self.controller.vel_I_gain[1] = aktion[2]
# self.controller.vel_I_gain[2] = aktion[3]

# self.controller.vel_D_gain[0] = aktion[4]
# self.controller.vel_D_gain[1] = aktion[4]
# self.controller.vel_D_gain[2] = aktion[5]

## Liveevaluation des Trainings
# Anzahl der evaluierten Epiosden
Evaluationsinterval = 2
# Schritte bis zur Evaluation
Evaluationsfrequenz = 10000

## Quadserial
# Serialport für Linux
serialname = "/dev/ttyACM2" 
# Aufräumen der Konsole nach jedem neuen Zustand
doclear = False
# Zustand auf der Konsole visualisieren
doprint = False

## Joystick
# Weiterleiten des Joystickzustandes an die serielle Schnittstelle
joystick2serial = True


# Schritte bis das Modell ein neue Parameter lernt
schritte_bis_update = 1
aktion_range_end2end = 300
aktion_mittelwert_end2end = 480

selbstorchestriert = True
selbstorchestriert_interval = [15, 31]