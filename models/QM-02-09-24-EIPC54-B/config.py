## Umweltparameter
# Schrittweite [s]
Schrittweite = 1e-3
# Initaler Zeitpunt [s]
Finaler_Zeitpunkt = 20
# Initaler Zeitpunkt [s]
Initaler_Zeitpunkt = 0

## Quadtest
# Verwenden der Default PID-Parameter
Quadtest_Default_PID = False

## Quadtrain
# Ordner des Modells
Ordnername = "QM-02-09-24-EIPC54-B"
# Modell fürs Training laden
Model_laden = False
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
Aktionsraum_EndeZuEnde = [1000, 1000, 1000, 1000, 10]
# Belohnungsgewichtung
belohnungsgewichtung = [0.0, 0.9, 0.1, 0.0]

## Quadpid
Aktionsinterval_PID = [5, 15]
# Zahl der zulernenden PID-Parameter
Aktionen = 18
# Aktionsperiode des PID-Reglers (PID-Parameter sind konstant) [s]
Updateperiode_PID = 1e-3
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
Evaluationsinterval = 2
# Schritte bis zur Evaluation
Evaluationsfrequenz = 10000

## Joystick
# Weiterleiten des Joystickzustandes an die serielle Schnittstelle
Joystick2Seriell = True











