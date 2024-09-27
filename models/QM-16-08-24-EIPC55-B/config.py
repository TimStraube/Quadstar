## Parameter
# NED = [Norden, Osten, Unten]
# NWU = [Norden, Westen, Oben]
# Schrittweite [s]
Schrittweite = 0.001
# Finaler Zeitpunt [s]
Finaler_Zeitpunkt = 1
# Initaler Zeitpunt [s]
Initaler_Zeitpunkt = 0

## Quadtest
# Verwenden der Default PID-Parameter
Quadtest_Default_PID = False

## Quadtrain
# Ordner des Modells
Ordnername = "QM-16-08-24-EIPC55-B"
# Modell fürs Training laden
Modell_laden = False
# Episodenzahl Training
Episoden = 1e9
# Rendern von Trainingsepisoden
render_modell = False 
# Architektur der neuronalen Netzwerke
pi = [2, 2]
vf = [2, 2]
# Lernrate
Lernrate = 1e-4
# Parallel Umwelten fürs Training
Parallele_Umwelten = 4
# Batchgröße
Batchmenge = 16

## Model
# Schritte bis das Modell ein neuen Parameter lernt
schritte_bis_update = 1
aktion_range_end2end = 300
aktion_mittelwert_end2end = 480
# Position, Geschwindigkeit, Drehlage, Drehgeschwindigkeit
belohnungsgewichtung = [0.0, 0.6, 0.2, 0.2]

## Quadpid
Aktionsinterval_PID = [0, 10]
# Zahl der zulernenden PID-Parameter
Aktionen = 6
# Updateperiode [s]
Updateperiode_PID = 0.001
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
Evaluationsinterval = 100
# Schritte bis zur Evaluation
Evaluationsfrequenz = 10000

## Joystick
# Weiterleiten des Joystickzustandes an die serielle Schnittstelle
joystick2serial = True


selbstorchestriert = True
selbstorchestriert_interval = [10, 15]