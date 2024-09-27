## Umweltparameter
# Schrittweite [s]
Schrittweite = 1e-3
# Initaler Zeitpunt [s]
Finaler_Zeitpunkt = 1
# Initaler Zeitpunkt [s]
Initaler_Zeitpunkt = 0

## Quadtest
# Verwenden der Default PID-Parameter
Quadtest_Default_PID = False

## Quadtrain
# Ordner des Modells
Ordnername = "QP-10-09-24-TPP52S-A"
# Modell fürs Training laden
Modell_laden = False
# Episodenzahl Training
Episoden = 1e9
# Rendern von Trainingsepisoden
Episoden_rendern = False 
# Architektur der neuronalen Netzwerke
Actor = [2, 2]
Critic = [2, 2]
# Lernrate
Lernrate = 1e-6
# Parallel Umwelten fürs Training
Parallele_Umwelten = 1
# Batchgröße
Batchmenge = 16

## Quadendezuende
# [Motor 1, Motor 2, Motor 3, Motor 4, Updateperiode [s]]
Aktionsraum_EndeZuEnde = [1000, 1000, 1000, 1000, 1]
# Belohnungsgewichtung
belohnungsgewichtung = [0.0, 1.0, 0.0, 0.0]

## Quadpid
Aktionsinterval_PID = [0, 10]
# Zahl der zulernenden PID-Parameter
Aktionen = 6
# Aktionsperiode des PID-Reglers (PID-Parameter sind konstant) [s]
Updateperiode_PID = 1e-2
# Auszug der zulernenden PID-Parameter
# self.controller.vel_P_gain[0] = aktion[0]
# self.controller.vel_P_gain[1] = aktion[0]

## Liveevaluation des Trainings
# Anzahl der evaluierten Epiosden
Evaluationsinterval = 2
# Schritte bis zur Evaluation
Evaluationsfrequenz = 10000

## Joystick
# Weiterleiten des Joystickzustandes an die serielle Schnittstelle
Joystick2Seriell = True










