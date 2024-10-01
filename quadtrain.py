import os
import torch
import shutil
import config
from quadpid import Quadpid
from quadend2end import Quadendezuende
from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import BaseCallback
from stable_baselines3.common.callbacks import EvalCallback
from stable_baselines3.common.callbacks import CallbackList
from stable_baselines3.common.logger import HParam

class HyperparameterParamCallback(BaseCallback):
    def _on_training_start(self):
        hparam_dict = {
            "algorithm": self.model.__class__.__name__,
            "learning rate": self.model.learning_rate,
            "gamma": self.model.gamma,
        }
        metric_dict = {
            "rollout/ep_len_mean": 0,
            "train/value_loss": 0.0,
        }
        self.logger.record(
            "hparams",
            HParam(hparam_dict, metric_dict),
            exclude=("stdout", "log", "json", "csv"),
        )

    def _on_step(self):
        return True

class Agent():
    def __init__(self):
        super(Agent, self).__init__()

        self.train()

    def train(self):
        # Vektorisiertes Environment
        if config.Ordnername[1] == "M":
            env = Quadendezuende()
            check_env(env)
            self.envs = make_vec_env(
                Quadendezuende, 
                seed = 4711, 
                n_envs = config.Parallele_Umwelten
            )
        if config.Ordnername[1] == "P":
            env = Quadpid()
            check_env(env)
            self.envs = make_vec_env(
                Quadpid, 
                seed = 4711, 
                n_envs = config.Parallele_Umwelten
            )
        # Architektur des Neuronalen Netzwerks
        policy_kwargs = dict(
            activation_fn = torch.nn.Tanh,
            net_arch = dict(
                pi = config.Actor, 
                vf = config.Critic
            )
        )
        # Proximal policy optimization Initialisierung
        self.model = PPO(
            "MlpPolicy",
            self.envs,
            tensorboard_log = (
                f"./models/{config.Ordnername}/tensorboard"
            ),
            policy_kwargs=policy_kwargs,
            learning_rate = config.Lernrate,
            batch_size = config.Batchmenge,
            gamma = 1.0,
            max_grad_norm = 10,
            verbose = 1
        )
        
        if config.Modell_laden: 
            model_path = (
                f"./models/{config.Ordnername}/main.zip"
            )
            self.model = PPO.load(model_path) 

        save_best_model_callback = EvalCallback(
            self.model.get_env(),
            eval_freq = config.Evaluationsfrequenz,
            best_model_save_path = (
                f"./models/{config.Ordnername}"
            ),
            log_path = os.path.join(
                f"./models/{config.Ordnername}", 
                "results"
            ),
            n_eval_episodes = config.Evaluationsinterval
        )

        shutil.copyfile(
            "./config.py", f"./models/{config.Ordnername}/config.py"
        )

        self.model.learn(
            config.Episoden,
            callback = CallbackList([
                HyperparameterParamCallback(),
                save_best_model_callback
            ]),
            progress_bar = True
        )

        self.model.save(f"./models/{config.Ordnername}/main")

if __name__ == "__main__":
    agent = Agent()