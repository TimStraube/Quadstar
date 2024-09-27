## Parameter
# NED = [Norden, Osten, Unten]
# NWU = [Norden, Westen, Oben]
Orientierung = "NED"
usePrecession = bool(False)
# Schrittweite [s]
Schrittweite = 0.001
# Initaler Zeitpunt [s]
Finaler_Zeitpunkt = 20
# Initaler Zeitpunkt [s]
Initaler_Zeitpunkt = 0

## Quadlive
#
port = 5001
# 
Quadlive_Schrittweite = 1

## Quadtest
# Schrittzahl bis Plot [s]
Quadtest_Finaler_Zeitpunkt = 5
# Verwenden der Default PID-Parameter
Quadtest_Default_PID = False

## Quadtrain
# Ordner des Modells 
Ordnername = "QM-06-09-24-EIPC53-B"
# Modell fürs Training laden
Model_laden = False
# Episodenzahl Training
Episoden = 1e9
# Finaler Zeitpunkt einer Trainingsepisode in s
Quadtrain_finaler_Zeitpunkt = 10
# Rendern von Trainingsepisoden
render_modell = False 
# Architektur der neuronalen Netzwerke
pi = [2, 2, 2, 2]
vf = [2, 2, 2, 2]

## Quadtrain
# Lernrate
lernrate = 1e-6
# Parallel Umwelten fürs Training
paralleleumwelten = 4
# Batchgröße
batchmenge = 16

## Quadendezuende
# [Motor 1, Motor 2, Motor 3, Motor 4, Updateperiode [s]]
Aktionsraum_EndeZuEnde = [1000, 1000, 1000, 1000, 1]
# Schritte bis das Modell einen neuen Parameter lernt
render_fortschrittsanzeige = True
# Position, Geschwindigkeit, Drehlage, Drehgeschwindigkeit
belohnungsgewichtung = [0.0, 0.1, 0.1, 0.8]

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