# Quadstar

## Targets of the project

1. Build a airworthy quadcopter from scratch which can be controlled with a joystick
2. Learn PID parameters in a simulation
3. Deploy the learned PID parameters and compare the performance to manually set ones
4. Learn a control policy 
5. Deploy the control policy and compare the performance to the PID controller

## Controller architectures

### PID controller

### End2end policy

## Drivetrain

### BLDC-motors

Axisflying C227 1960KV FPV Motor schwarz

### BLDC-driver

B-G431B-ESC1 

### Akkumulator

## Information flow system

### Main flight computer

The core of the system is a STM32F411RE microcontroller which computes all processes required for the flight controller which includes sensor fusion.

### Sensor extension board

X-NUCLEO-IKS4A1

## Simulation

### Configuration

All volatile parameters on the user side can be controlled from a single file.
When a new model is trained the current config is used for training and is saved in the SQLite database models/config.db.

### Plot of quadcopter trajectories

The program quadtest.py can be used to create matplotlib-plots of position, velocity and attitude trajectories from simulation:

```python3 quadtest.py```

### Visualize the quadcopter in the browser with flask

Quadlive provides two functionalities.

```python3 quadlive.py```

### Train models with Stable-Baselines3

To train the a quadcopter flight controller on the current configuration run ```python3 quadtrain.py```.

## Delopyment

### Joystick

To sample joystick values and send them to the main flight computer run ```python3 joystick.py```.

## Credit

The PID controller as well as the flight dynamic model inplementation from [quadcopter_simcon](https://github.com/bobzwik/Quadcopter_SimCon) have been modified for this reposiory. Big thanks to the creator.  
