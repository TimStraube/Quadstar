## Parameter
# NED = [Norden, Osten, Unten]
# NWU = [Norden, Westen, Oben]
# Schrittweite [s]
Schrittweite = 0.001
# Initaler Zeitpunt [s]
<<<<<<< HEAD
Finaler_Zeitpunkt = 5
=======
Finaler_Zeitpunkt = 1
>>>>>>> 774eadd08092c7a61d75932ae93d41c1ef60ee2c
# Initaler Zeitpunkt [s]
Initaler_Zeitpunkt = 0

## Quadtest
# Verwenden der Default PID-Parameter
Quadtest_Default_PID = False

## Quadtrain
# Ordner des Modells 
<<<<<<< HEAD
Ordnername = "QP-12-09-24-EIPC55-A"
=======
Ordnername = "QM-16-08-24-EIPC55-E"
>>>>>>> 774eadd08092c7a61d75932ae93d41c1ef60ee2c
# Modell fürs Training laden
Modell_laden = False
# Episodenzahl Training
Episoden = 1e9
# Rendern von Trainingsepisoden
render_modell = False 
# Architektur der neuronalen Netzwerke
Actor = [2, 2]
Critic = [2, 2]
<<<<<<< HEAD

## Quadtrain
# Lernrate
Lernrate = 1e-6
# Parallel Umwelten fürs Training
Parallele_Umwelten = 1
=======
# Lernrate
Lernrate = 1e-4
# Parallel Umwelten fürs Training
Parallele_Umwelten = 4
>>>>>>> 774eadd08092c7a61d75932ae93d41c1ef60ee2c
# Batchgröße
Batchmenge = 16

## Quadendezuende
# [Motor 1, Motor 2, Motor 3, Motor 4, Updateperiode [s]]
Aktionsraum_EndeZuEnde = [1000, 1000, 1000, 1000, 1]
# Position, Geschwindigkeit, Drehlage, Drehgeschwindigkeit
Belohnungsgewichtung = [0.0, 0.6, 0.2, 0.2]

## Quadpid
Aktionsinterval_PID = [1e-2, 10]
# Zahl der zulernenden PID-Parameter
Aktionen = 2
# Updateperiode [s]
Updateperiode_PID = 0.01
# Auszug der zulernenden PID-Parameter
# self.controller.vel_P_gain[0] = aktion[0]
# self.controller.vel_P_gain[1] = aktion[0]
# self.controller.vel_P_gain[2] = aktion[1]

## Liveevaluation des Trainings
# Anzahl der evaluierten Epiosden
Evaluationsinterval = 2
# Schritte bis zur Evaluation
<<<<<<< HEAD
Evaluationsfrequenz = 25

## Quadserial
# Serialport für Linux
serialname = "/dev/ttyACM2" 
# Aufräumen der Konsole nach jedem neuen Zustand
doclear = False
# Zustand auf der Konsole visualisieren
doprint = False
=======
Evaluationsfrequenz = 10000
>>>>>>> 774eadd08092c7a61d75932ae93d41c1ef60ee2c

## Joystick
# Weiterleiten des Joystickzustandes an die serielle Schnittstelle
joystick2serial = True


# Schritte bis das Modell ein neue Parameter lernt
schritte_bis_update = 1
aktion_range_end2end = 300
aktion_mittelwert_end2end = 480

selbstorchestriert = True
selbstorchestriert_interval = [15, 31]