# Quadstar

## Targets of the project

1. Build a airworthy quadcopter from scratch which can be controlled with a joystick
2. Learn PID parameters in a simulation
3. Deploy the learned PID parameters and compare the performance to manually set ones
4. Learn a control policy 
5. Deploy the control policy and compare the performance to the PID controller

## Controller architecture

### PID controller

### End2end policy

## Hardware

### BLDC-motors

Axisflying C227 1960KV FPV Motor schwarz

### BLDC-driver

### Microcontroller

STM32F411RE

### Sensor extension board

X-NUCLEO-IKS4A1

### Akkumulator

## Simulation

### Config

All volatile parameters on the user side can be controlled from a single file.
When a new model is trained the current config is used for training and is saved in the SQLite database models/config.db.

### Plot of quadcopter trajectories

The program quadtest.py can be used to create matplotlib-plots of position, velocity and attitude trajectories from simulation:

```
python3 quadtest.py
```

### Visualize the quadcopter in the browser with flask

Quadlive provides two functionalities.

```
python3 quadlive.py 
```

### Train models with Stable-Baselines3

```
python3 quadtrain.py
```

## Delopyment

### Joystick

```
python3 joystick.py
```

## Credit

The PID controller as well as the flight dynamic model inplementation from [quadcopter_simcon](https://github.com/bobzwik/Quadcopter_SimCon) have been modified for this reposiory. Big thanks to the creator.  
