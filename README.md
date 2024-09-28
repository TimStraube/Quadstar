# Quadstar

### Targets of the project

1. Develop a quadcopter from scratch
2. Utilize deep reinforcement learning for learning a control policy

### Config

All volatile parameters on the user side can be controlled from a single file.
When a new model is trained the current config is used for training and is saved in the respective model folder.

### Plot of quadcopter trajectories

The program quadtest.py can be used to create matplotlib-plots of position and attitude trajectories from simulation:

```
python3 quadtest.py
```

### Visualize the quadcopter in the browser with flask

```
python3 quadlive.py 
```

### Train models with Stable-Baselines3

```
python3 quadtrain.py
```

### Joystick

```
python3 joystick.py
```
