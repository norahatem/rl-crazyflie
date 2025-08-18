# train.py
import os
from datetime import datetime
import numpy as np
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.env_util import make_vec_env
from stable_baselines3.common.callbacks import EvalCallback, CheckpointCallback

from gym_pybullet_drones.envs.WaypointsAviary import WayPointsAviary
from gym_pybullet_drones.utils.enums import ObservationType, ActionType

ALGORITHM = "SAC"

DEFAULT_OBS = ObservationType('kin') # 'kin' or 'rgb'
DEFAULT_ACT = ActionType('rpm') # 'rpm' or 'pid' or 'vel' or 'one_d_rpm' or 'one_d_pid'

DEFAULT_OUTPUT_FOLDER = 'results'


def train(output_folder=DEFAULT_OUTPUT_FOLDER, total_timesteps=int(1e7)):
    """
    Train a reinforcement learning model using PPO or SAC on the HoverAviary environment.

    Parameters:
        output_folder (str): Path to the folder where results will be stored.
        total_timesteps (int): Total number of training steps.
    """
    # Create a unique directory for this training run
    timestamp = datetime.now().strftime("%m.%d.%Y_%H.%M.%S")
    run_dir = os.path.join(output_folder, 'save-' + timestamp)
    os.makedirs(run_dir, exist_ok=True)

    # Create training and evaluation environments
    train_env = make_vec_env(WayPointsAviary,
                             env_kwargs=dict(obs=DEFAULT_OBS, act=DEFAULT_ACT, gui=False),
                             n_envs=10,
                             seed=0)
    eval_env = WayPointsAviary(obs=ObservationType.KIN, act=ActionType.RPM, gui=False)

    print('[INFO] Action space:', train_env.action_space)
    print('[INFO] Observation space:', train_env.observation_space)

    # Initialize the model with the selected algorithm
    if ALGORITHM == "PPO":
        model = PPO('MlpPolicy',
                    train_env,
                    tensorboard_log=run_dir + '/tb/',
                    verbose=1, device="cuda")
    elif ALGORITHM == "SAC":
        model = SAC('MlpPolicy',
                    train_env,
                    tensorboard_log=run_dir + '/tb/',
                    verbose=1, device= "cuda")
    else:
        print("[ERROR] Model algorithm is not supported")
        return 
        
    
    # Set up evaluation callback to periodically evaluate and save best model
    eval_callback = EvalCallback(eval_env,
                                 verbose=1,
                                 best_model_save_path=run_dir,
                                 log_path=run_dir,
                                 eval_freq=1000,
                                 deterministic=True)
    
    # Save a checkpoint every 100,000 steps
    checkpoint_callback = CheckpointCallback(
        save_freq=100_000,
        save_path=os.path.join(run_dir, 'checkpoints'),
        name_prefix=f'{ALGORITHM}_checkpoint',
        save_replay_buffer=True,     # Only needed for off-policy algos like SAC
        save_vecnormalize=True
    )
    
    callbacks = [eval_callback, checkpoint_callback]

    # Start training
    model.learn(total_timesteps=total_timesteps,
                callback=callbacks,
                log_interval=100)

    # Save final model
    model.save(os.path.join(run_dir, 'final_model.zip'))
    print("[INFO] Training completed. Models saved to:", run_dir)

    # Print evaluation log
    eval_file = os.path.join(run_dir, 'evaluations.npz')
    if os.path.exists(eval_file):
        with np.load(eval_file) as data:
            for t, r in zip(data['timesteps'], data['results']):
                print(f"{t}, {r[0]}")

if __name__ == '__main__':
    train(total_timesteps=3e7)
