"""
author: Tim Leonard Straube
email: hi@optimalpi.de
"""

import os
import torch
import shutil
import config
import sqlite3
import json
from quadpid import Quadpid
from quadend2end import Quadend2end
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
        if config.model_id[1] == "M":
            env = Quadend2end()
            check_env(env)
            self.envs = make_vec_env(
                Quadend2end, 
                seed = 4711, 
                n_envs = config.parallel_environments
            )
        if config.model_id[1] == "P":
            env = Quadpid()
            check_env(env)
            self.envs = make_vec_env(
                Quadpid, 
                seed = 4711, 
                n_envs = config.parallel_environments
            )
        # Architektur des Neuronalen Netzwerks
        policy_kwargs = dict(
            activation_fn = torch.nn.Tanh,
            net_arch = dict(
                pi = config.actor, 
                vf = config.critic
            )
        )
        # Proximal policy optimization Initialisierung
        self.model = PPO(
            "MlpPolicy",
            self.envs,
            tensorboard_log = (
                f"./models/{config.model_id}/tensorboard"
            ),
            policy_kwargs=policy_kwargs,
            learning_rate = config.learning_rate,
            batch_size = config.batchsize,
            gamma = 1.0,
            max_grad_norm = 10,
            verbose = 1
        )
        
        if config.load_model: 
            model_path = (
                f"./models/{config.model_id}/main.zip"
            )
            self.model = PPO.load(model_path) 

        save_best_model_callback = EvalCallback(
            self.model.get_env(),
            eval_freq = config.evaluation_frequency,
            best_model_save_path = (
                f"./models/{config.model_id}"
            ),
            log_path = os.path.join(
                f"./models/{config.model_id}", 
                "results"
            ),
            n_eval_episodes = config.evaluation_step_size
        )

        conn = None
        try:
            # Ensure the models folder exists
            os.makedirs("models", exist_ok=True)

            # Path to the database file
            database_path = os.path.join(
                "models", 
                "config.db"
            )

            conn = sqlite3.connect(database_path)
            cursor = conn.cursor()
            # Create table with name based on model_id
            table_name = f"model_{config.model_id.replace('-', '_')}" 
            create_table_query = f"""
                CREATE TABLE IF NOT EXISTS {table_name} (
                    step_size FLOAT,
                    episode_end_time FLOAT,
                    episode_start_time FLOAT,
                    load_model BOOLEAN,
                    episodes BIGINT,
                    actor JSON,
                    critic JSON,
                    learning_rate FLOAT,
                    parallel_environments INT,
                    batchsize INT,
                    aktionspace_end2end JSON,
                    reward_weights JSON,
                    actionspace_pid JSON,
                    actions INT,
                    pid_values_update_step_size FLOAT,
                    evaluation_step_size INT
                )
            """
            cursor.execute(create_table_query)
            
            # Insert model metadata and file path
            insert_query = f"""
                INSERT INTO {table_name} (
                    step_size,
                    episode_end_time,
                    episode_start_time,
                    load_model,
                    episodes,
                    actor,
                    critic,
                    learning_rate,
                    parallel_environments,
                    batchsize,
                    actionspace_end2end,
                    reward_weights,
                    actionspace_pid,
                    actions,
                    pid_values_update_step_size,
                    evaluation_step_size
                ) VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)
            """
            cursor.execute(
                insert_query, 
                (
                    config.step_size,
                    config.episode_end_time,
                    config.episode_start_time,
                    config.load_model,
                    config.episodes,
                    json.dumps(config.actor),
                    json.dumps(config.critic),
                    config.learning_rate,
                    config.parallel_environments,
                    config.batchsize,
                    json.dumps(config.actionspace_end2end),
                    json.dumps(config.reward_weights),
                    json.dumps(config.actionspace_pid),
                    config.actions,
                    config.pid_values_update_step_size,
                    config.evaluation_step_size
                )
            )
            
            # Commit the transaction
            conn.commit()
        except sqlite3.Error as e:
            print("SQLite error: " + str(e))

        self.model.learn(
            config.episodes,
            callback = CallbackList([
                HyperparameterParamCallback(),
                save_best_model_callback
            ]),
            progress_bar = True
        )

        self.model.save(f"./models/{config.model_id}/main")

if __name__ == "__main__":
    agent = Agent()