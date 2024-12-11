## Parameter
# NED = [North, East, Down]
# [s]
step_size = 0.001
# [s]
episode_end_time = 20
# [s]
episode_start_time = 0

## Quadlive
#
port = 5001
# 
quadlive_step_size = 1

## Quadtest
# [s]
quadtest_final_time = 5
# Use default PID parameters
quadtest_default_pid = False

## Quadtrain
# Foldername and id for the model
model_id = "QM-06-09-24-EIPC53-B"
# Load model for continued training
load_model = False
# Maximal number of episodes
episodes = 1e9
# Network architecture
actor = [2, 2, 2, 2]
critic = [2, 2, 2, 2]
# Gradient descent learning rate
learning_rate = 1e-6
# Parallel environments
parallel_environments = 4
# Batchsize
batchsize = 16

## Quadend2end
# [Max motor rps [rad/s], Update step size [s]]
actionspace_end2end = [1000, 1000, 1000, 1000, 1]
# Position, velocity, attitude, angular rate
reward_weights = [0.0, 0.1, 0.1, 0.8]

## Quadpid
# PID-parameter interval
actionspace_pid = [0, 10]
# Number of PID-parameters which are controlled
actions = 6
# [s]
pid_values_update_step_size = 0.01
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
evaluation_step_size = 2
# Schritte bis zur Evaluation
evaluation_frequency = 10000