## Parameter
# NED = [Norden, Osten, Unten]
# NWU = [Norden, Westen, Oben]
# Schrittweite [s]
Schrittweite = 0.001
# Initaler Zeitpunt [s]
Finaler_Zeitpunkt = 2
# Initaler Zeitpunkt [s]
Initaler_Zeitpunkt = 0

## Quadtest
# Verwenden der Default PID-Parameter
Quadtest_Default_PID = False

## Quadtrain
# Ordner des Modells 
Ordnername = "QP-12-09-24-EIPC53-C"
# Modell fürs Training laden
Modell_laden = False
Modell_laden = False
# Episodenzahl Training
Episoden = 1e8
# Rendern von Trainingsepisoden
render_modell = False 
# Architektur der neuronalen Netzwerke
Actor = [2, 2]
Critic = [2, 2]

## Quadtrain
# Lernrate
Lernrate = 1e-6
# Parallel Umwelten fürs Training
Parallele_Umwelten = 1
# Batchgröße
Batchmenge = 16
Batchmenge = 16

## Quadendezuende
# [Motor 1, Motor 2, Motor 3, Motor 4, Updateperiode [s]]
Aktionsraum_EndeZuEnde = [1000, 1000, 1000, 1000, 1]
# Position, Geschwindigkeit, Drehlage, Drehgeschwindigkeit
belohnungsgewichtung = [0.0, 1.0, 0.0, 0.0]

## Quadpid
Aktionsinterval_PID = [1e-2, 10]
# Zahl der zulernenden PID-Parameter
Aktionen = 1
# Updateperiode [s]
Updateperiode_PID = 0.01
# Auszug der zulernenden PID-Parameter
# self.controller.vel_P_gain[0] = aktion[0]
# self.controller.vel_P_gain[1] = aktion[0]
# self.controller.vel_P_gain[2] = aktion[1]

## Liveevaluation des Trainings
# Anzahl der evaluierten Epiosden
Evaluationsinterval = 2
Evaluationsinterval = 2
# Schritte bis zur Evaluation
Evaluationsfrequenz = 25

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