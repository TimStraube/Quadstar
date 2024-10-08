## Parameter
# NED = [North, East, Down]
# [s]
step_size = 0.001
# [s]
episode_end_time = 4
# [s]
episode_start_time = 0

## Quadtrain
# Foldername and id for the model
model_id = "QP-07-10-24-TPP52S-C"
# Load model for continued training
load_model = False
# Maximal number of episodes
episodes = 1e8
# Network architecture
actor = [2, 2]
critic = [2, 2]
# Gradient descent learning rate
learning_rate = 1e-4
# Parallel environments
parallel_environments = 4
# Batchsize
batchsize = 16

## Quadend2end
# [Max motor rps [rad/s], Update step size [s]]
aktionspace_end2end = [1000, 1]
# Position, velocity, attitude, angular rate
reward_weights = [0.0, 0.4, 0.2, 0.2]

## Quadpid
# PID-parameter interval
aktionspace_pid = [0, 10]
# Number of PID-parameters which are controlled
aktions = 6
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