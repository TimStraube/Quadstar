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
# Schrittzahl bis Plot
Quadtest_Schrittzahl = 1000
# Verwenden der Default PID-Parameter
Quadtest_Default_PID = False

## Quadtrain
# Ordner des Modells 
Ordnername = "QP-28-08-24-EIPC55-E"
# Modell fürs Training laden
Model_laden = False
# Episodenzahl Training
Episoden = 1e9
# Finaler Zeitpunkt einer Trainingsepisode in s
Quadtrain_finaler_Zeitpunkt = 1
# Rendern von Trainingsepisoden
render_modell = False 
# Architektur der neuronalen Netzwerke
pi = [8, 8]
vf = [8, 8]

## Quadtrain
# Lernrate
lernrate = 1e-2
# Parallel Umwelten fürs Training
paralleleumwelten = 1
# Batchgröße
batchmenge = 16

## Quadendezuende
# [Motor 1, Motor 2, Motor 3, Motor 4, Schrittweite]
Aktionsraum_EndeZuEnde = [10, 10, 10, 10, 1]
# Schritte bis das Modell einen neuen Parameter lernt
render_fortschrittsanzeige = True
# Position, Geschwindigkeit, Drehlage, Drehgeschwindigkeit
belohnungsgewichtung = [0.0, 0.4, 0.3, 0.3]

## Quadpid
Aktionsinterval_PID = [4.9, 4]
# Zahl der zulernenden PID-Parameter
aktionen = 18
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
# self.controller.attitute_p_gain[0] = aktion[6]
# self.controller.attitute_p_gain[1] = aktion[6]
# self.controller.attitute_p_gain[2] = aktion[7]
# self.controller.Pp = aktion[8]
# self.controller.Dp = aktion[9]
# self.controller.Pq = self.controller.Pp
# self.controller.Dq = self.controller.Dp
# self.controller.Pr = aktion[10]
# self.controller.Dr = aktion[11]
# self.controller.rate_P_gain[0] = aktion[12]
# self.controller.rate_P_gain[1] = aktion[13]
# self.controller.rate_P_gain[2] = aktion[14]
# self.controller.rate_D_gain[0] = aktion[15]
# self.controller.rate_D_gain[1] = aktion[16]
# self.controller.rate_D_gain[2] = aktion[17]

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




