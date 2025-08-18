# evaluate.py
import os
import time
import argparse
import numpy as np
from stable_baselines3 import PPO, SAC
from stable_baselines3.common.evaluation import evaluate_policy

from gym_pybullet_drones.envs.WaypointsAviary import WayPointsAviary
from gym_pybullet_drones.utils.enums import ObservationType, ActionType
from gym_pybullet_drones.utils.Logger import Logger
from gym_pybullet_drones.utils.utils import sync, str2bool

DEFAULT_OBS = ObservationType('kin') # 'kin' or 'rgb'
DEFAULT_ACT = ActionType('rpm') # 'rpm' or 'pid' or 'vel' or 'one_d_rpm' or 'one_d_pid'
DEFAULT_OUTPUT_FOLDER = 'results'

ALGORITHM = "SAC"

RANDOM = True
targets = np.array([[0,0,0],[1,1,1],[2,2,2]])


def evaluate(model_path, gui=True, record_video=True, plot=True, output_folder=DEFAULT_OUTPUT_FOLDER):
    """
    Evaluate a trained model on the WaypointsAviary environment.

    Parameters:
        model_path (str): Path to the saved model (.zip).
        gui (bool): Whether to enable PyBullet GUI.
        record_video (bool): Whether to record video during the rollout.
        plot (bool): Whether to plot the flight path after evaluation.
        output_folder (str): Where to save logs and plots.
    """
    
    # Load the trained model from file
    if not os.path.isfile(model_path):
        print("[ERROR] Model file not found:", model_path)
        return

    if ALGORITHM == "PPO":
        model = PPO.load(model_path, device="cuda")
    elif ALGORITHM == "SAC":
        model = SAC.load(model_path, device="cuda")
    else:
        print("[ERROR] Model algorithm is not supported")
        return  
    
    # Create two environments:
    # - One with GUI and video recording for rendering
    # - One headless (no GUI) for performance evaluation
    test_env = WayPointsAviary(obs=DEFAULT_OBS, act=DEFAULT_ACT, gui=gui,record=record_video, initial_xyzs= np.array([[1,1,1]]), flight_dome_size=4.0, num_targets=4, max_duration_seconds=25, random_mode=RANDOM, custom_waypoints=targets)

     # Initialize the flight logger to log drone states for later visualization
    logger = Logger(logging_freq_hz=int(test_env.CTRL_FREQ),
                    num_drones=1,
                    output_folder=output_folder,
                    colab=False)

    # Reset the GUI environment and start a rollout
    obs, info = test_env.reset()
    start = time.time()

    
    for i in range(int((test_env.EPISODE_LEN_SEC) * test_env.CTRL_FREQ)):
        # Get action from trained model (deterministic policy)
        action, _ = model.predict(obs, deterministic=True)

        # Take a step in the environment
        obs, reward, terminated, truncated, info = test_env.step(action)

        # Flatten observation and action arrays for logging
        obs2 = obs.squeeze()
        act2 = action.squeeze()

        # Print step-level details
        print(f"Step {i}: Reward={reward}, Terminated={terminated}, Truncated={truncated}")

        # Log state data for plotting (only for 'kin' observation)
        if DEFAULT_OBS == ObservationType.KIN:
            logger.log(drone=0,
                    timestamp=i / test_env.CTRL_FREQ,
                    state=np.hstack([
                        obs2[0:3],      # Position (x, y, z)
                        np.zeros(4),    # Quaternion placeholder
                        obs2[3:15],     # Velocities and angular rates
                        act2            # Control input
                    ]),
                    control=np.zeros(12))  # Placeholder control vector

        # Render simulation (if GUI enabled)
        test_env.render()

        # Keep simulation in real time
        sync(i, start, test_env.CTRL_TIMESTEP)

        # Restart episode if drone is terminated (crashed/out of bounds)
        if terminated or truncated:
            done = True
            print(f"info: {info}")
            obs, info = test_env.reset()

    test_env.close()
    logger.export_positions_csv("crazyflie_trajectory.csv")

    # Plot flight trajectory if requested
    if plot and DEFAULT_OBS == ObservationType.KIN:
        logger.plot()

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Evaluate a trained model for Waypoints Aviary")

    # CLI argument for model path (required)
    parser.add_argument('--model_path', type=str, required=True,
                        help="Path to the trained model (.zip)")

    # Toggle PyBullet GUI
    parser.add_argument('--gui', type=str2bool, default=True,
                        help="Whether to enable PyBullet GUI (default: True)")

    # Toggle video recording
    parser.add_argument('--record_video', type=str2bool, default=False,
                        help="Whether to record video of evaluation (default: False)")

    # Toggle plot generation
    parser.add_argument('--plot', type=str2bool, default=False,
                        help="Whether to plot the trajectory after evaluation (default: False)")

    # Output folder for logs and plots
    parser.add_argument('--output_folder', type=str, default=DEFAULT_OUTPUT_FOLDER,
                        help="Output folder for logs/plots (default: results/)")

    # Parse arguments and run evaluation
    args = parser.parse_args()
    evaluate(**vars(args))