## Parameter
# NED = [Norden, Osten, Unten]
# NWU = [Norden, Westen, Oben]
# Schrittweite [s]
Schrittweite = 0.001
# Initaler Zeitpunt [s]
Finaler_Zeitpunkt = 20
# Initaler Zeitpunkt [s]
Initaler_Zeitpunkt = 0

## Quadtest
# Verwenden der Default PID-Parameter
Quadtest_Default_PID = False

## Quadtrain
# Ordner des Modells 
Ordnername = "QM-16-08-24-EIPC55-A"
# Modell fürs Training laden
Modell_laden = False
# Episodenzahl Training
Episoden = 1e9
# Finaler Zeitpunkt einer Trainingsepisode in s
Quadtrain_finaler_Zeitpunkt = 1
# Rendern von Trainingsepisoden
render_modell = False 
# Architektur der neuronalen Netzwerke
Actor = [2, 2]
Critic = [2, 2]
# Lernrate
Lernrate = 1e-4
# Parallel Umwelten fürs Training
Parallele_Umwelten = 4
# Batchgröße
Batchmenge = 16

## Quadendezuende
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
evaluationsinterval = 2
# Schritte bis zur Evaluation
evaluationsfrequenz = 10000

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