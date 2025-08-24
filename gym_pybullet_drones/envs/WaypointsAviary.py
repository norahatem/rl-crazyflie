import numpy as np

from gym_pybullet_drones.envs.BaseRLAviary import BaseRLAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics, ActionType, ObservationType
from gym_pybullet_drones.utils.waypoint_handler import WaypointHandler
from gymnasium import spaces
# from pybullet_utils import bullet_client
import pybullet as p
import os

class WayPointsAviary(BaseRLAviary):
    def __init__(self,
                 drone_model: DroneModel=DroneModel.CF2X,
                 initial_xyzs=np.array([[1,1,1]]),
                 initial_rpys=None,
                 use_yaw_targets: bool = False,
                 goal_reach_distance: float = 0.05,
                 goal_reach_angle: float = 0.1,
                 num_targets: int = 4,
                 flight_dome_size: float = 3.0,
                 max_duration_seconds: float = 30.0,
                 physics: Physics=Physics.PYB,
                 pyb_freq: int = 240,
                 ctrl_freq: int = 30,
                 gui=False,
                 record=False,
                 obs: ObservationType=ObservationType.KIN,
                 act: ActionType=ActionType.RPM,
                 random_mode: bool = True,
                 custom_waypoints: np.ndarray = None,
                 custom_yaw_targets: np.ndarray = None
                 ):
        
        self.EPISODE_LEN_SEC = max_duration_seconds
        self.FLIGHT_DOME_SIZE = flight_dome_size
        self.USE_YAW_TARGET = use_yaw_targets
        self.ENABLE_RENDER = gui  # Only render targets if GUI is enabled
        
        super().__init__(drone_model=drone_model,
                    num_drones=1,
                    initial_xyzs=initial_xyzs,
                    initial_rpys=initial_rpys,
                    physics=physics,
                    pyb_freq=pyb_freq,
                    ctrl_freq=ctrl_freq,
                    gui=gui,
                    record=record,
                    obs=obs,
                    act=act
                    )
        
        # self._p = bullet_client.BulletClient()
        # self._p._client = self.CLIENT  # ONLY set client, don't reconnect
        
        self.waypoints = WaypointHandler(
            num_targets=num_targets,
            use_yaw_targets=use_yaw_targets,
            goal_reach_distance=goal_reach_distance,
            goal_reach_angle=goal_reach_angle,
            flight_dome_size=flight_dome_size,
            min_height=0.1,
            np_random=np.random.default_rng(),
            client_id=self.CLIENT,
            random_mode=random_mode,
            waypoints=custom_waypoints,
            yaw_waypoints=custom_yaw_targets,
        )
        # self.waypoints.p = self.getPyBulletClient() 
        # self.waypoints.p = self._p #should have physice client?
        self._just_advanced_target = 0
        self.target_visual_ids = []
        file_dir = os.path.dirname(os.path.realpath(__file__))
        self.target_urdf_path = os.path.join(file_dir, "../assets/target.urdf")
        # Fallback path if needed
        if not os.path.exists(self.target_urdf_path):
            self.target_urdf_path = "D:/Uni/ENGAGE/gym-pybullet-drones/gym_pybullet_drones/assets/target.urdf"

    
    def reset(self, seed = None, options = None):
        obs, info = super().reset(seed, options)
        self.waypoints.reset(np.random.default_rng())
        self._render_targets()
        print("Targets after reset:", self.waypoints.targets)
        
        # for i, pos in enumerate(self.waypoints.targets):
        #     self._p.addUserDebugText(f"W{i}", pos, textColorRGB=[1, 0, 0], textSize=1.5, lifeTime=200)
        #     self._p.addUserDebugLine(pos, [pos[0], pos[1], pos[2] + 0.2], [0, 1, 0], lineWidth=2, lifeTime=2000)
        return obs, info
    
    def _render_targets(self):
        """Handle rendering of target waypoints."""
        # Clear existing visual targets
        self._clear_target_visuals()
        
        if self.ENABLE_RENDER and len(self.waypoints.targets) > 0:
            self.target_visual_ids = []
            
            for i, target in enumerate(self.waypoints.targets):
                try:
                    visual_id = p.loadURDF(
                        self.target_urdf_path,
                        basePosition=target,
                        useFixedBase=True,
                        globalScaling=self.waypoints.goal_reach_distance / 4.0,
                        physicsClientId=self.CLIENT,
                    )
                    self.target_visual_ids.append(visual_id)
                    
                    # Color targets with gradient (green to yellow)
                    color_intensity = 1 - (i / len(self.waypoints.targets))
                    p.changeVisualShape(
                        visual_id,
                        linkIndex=-1,
                        rgbaColor=(color_intensity, 0, 0, 1),
                        physicsClientId=self.CLIENT,
                    )
                except Exception as e:
                    print(f"Warning: Could not load target visual {i}: {e}")
                    
    def _clear_target_visuals(self):
        """Clear all target visual elements."""
        for visual_id in self.target_visual_ids:
            try:
                p.removeBody(visual_id, physicsClientId=self.CLIENT)
            except Exception:
                pass  # Body might already be removed
        self.target_visual_ids = []
        
    def _update_target_visuals(self):
        """Update target visuals when a target is reached."""
        if not self.ENABLE_RENDER:
            return
            
        # Remove the first target visual
        if len(self.target_visual_ids) > 0:
            try:
                p.removeBody(self.target_visual_ids[0], physicsClientId=self.CLIENT)
            except Exception:
                pass
            self.target_visual_ids = self.target_visual_ids[1:]
            
            # Recolor remaining targets
            for i, visual_id in enumerate(self.target_visual_ids):
                color_intensity = 1 - (i / len(self.target_visual_ids)) if len(self.target_visual_ids) > 0 else 1
                try:
                    p.changeVisualShape(
                        visual_id,
                        linkIndex=-1,
                        rgbaColor=(color_intensity,0 , 0, 1),
                        physicsClientId=self.CLIENT,
                    )
                except Exception:
                    pass
    
    def _observationSpace(self):
        if self.OBS_TYPE == ObservationType.KIN:
            ############################################################
            #### OBS SPACE OF SIZE 12
            #### Observation vector ### X        Y        Z       Q1   Q2   Q3   Q4   R       P       Y       VX       VY       VZ       WX       WY       WZ
            lo = -np.inf
            hi = np.inf
            lo_xy = -2*self.FLIGHT_DOME_SIZE
            hi_xyz = 2*self.FLIGHT_DOME_SIZE
            obs_lower_bound = np.array([[lo_xy,lo_xy,0, lo,lo,lo,lo,lo,lo,lo,lo,lo] for i in range(self.NUM_DRONES)])
            obs_upper_bound = np.array([[hi_xyz,hi_xyz,hi_xyz,hi,hi,hi,hi,hi,hi,hi,hi,hi] for i in range(self.NUM_DRONES)])
            #### Add action buffer to observation space ################
            act_lo = -1
            act_hi = +1
            for i in range(self.ACTION_BUFFER_SIZE):
                if self.ACT_TYPE in [ActionType.RPM, ActionType.VEL]:
                    obs_lower_bound = np.hstack([obs_lower_bound, np.array([[act_lo,act_lo,act_lo,act_lo] for i in range(self.NUM_DRONES)])])
                    obs_upper_bound = np.hstack([obs_upper_bound, np.array([[act_hi,act_hi,act_hi,act_hi] for i in range(self.NUM_DRONES)])])
                elif self.ACT_TYPE==ActionType.PID:
                    obs_lower_bound = np.hstack([obs_lower_bound, np.array([[act_lo,act_lo,act_lo] for i in range(self.NUM_DRONES)])])
                    obs_upper_bound = np.hstack([obs_upper_bound, np.array([[act_hi,act_hi,act_hi] for i in range(self.NUM_DRONES)])])
                elif self.ACT_TYPE in [ActionType.ONE_D_RPM, ActionType.ONE_D_PID]:
                    obs_lower_bound = np.hstack([obs_lower_bound, np.array([[act_lo] for i in range(self.NUM_DRONES)])])
                    obs_upper_bound = np.hstack([obs_upper_bound, np.array([[act_hi] for i in range(self.NUM_DRONES)])])
            if self.USE_YAW_TARGET:
                delta_dim = 4
            else:
                delta_dim = 3
            delta_lo = np.full((self.NUM_DRONES, delta_dim), -np.inf)
            delta_hi = np.full((self.NUM_DRONES, delta_dim), np.inf)
            obs_lower_bound = np.hstack([obs_lower_bound, delta_lo])
            obs_upper_bound = np.hstack([obs_upper_bound, delta_hi])
            return spaces.Box(low=obs_lower_bound, high=obs_upper_bound, dtype=np.float32)
        else:
            return super()._observationSpace()
        
    def _computeObs(self):
        if self.OBS_TYPE == ObservationType.KIN:
            ############################################################
            #### OBS SPACE OF SIZE 15?
            obs_15 = np.zeros((self.NUM_DRONES,15))
            for i in range(self.NUM_DRONES):
                #obs = self._clipAndNormalizeState(self._getDroneStateVector(i))
                obs = self._getDroneStateVector(i)
                drone_ang = obs[7:10]
                drone_pos = obs[0:3]
                drone_quat = obs[3:7]
                target_deltas = self.waypoints.distance_to_targets(
                    ang_pos=drone_ang,
                    lin_pos=drone_pos,
                    quaternion=drone_quat
                )
                target_delta = target_deltas[0] if len(target_deltas) > 0 else np.zeros(3)
                obs_15[i, :] = np.hstack([drone_pos, drone_ang, obs[10:13], obs[13:16], target_delta]).reshape(15,)
            ret = np.array([obs_15[i, :] for i in range(self.NUM_DRONES)]).astype('float32')
            #### Add action buffer to observation #######################
            for i in range(self.ACTION_BUFFER_SIZE):
                ret = np.hstack([ret, np.array([self.action_buffer[i][j, :] for j in range(self.NUM_DRONES)])])
            return ret.astype(np.float32)
        return super()._computeObs()

    def _computeReward(self):
        ret = -0.1
        state = self._getDroneStateVector(0)
        if np.linalg.norm(state[0:3]) > self.FLIGHT_DOME_SIZE:
            ret = -100
            
        current_target = self.waypoints.current_target
        # print(f"comp reward, current target is: {current_target}")
        if current_target is not None:
            target_z = current_target[2]
            #penalise crashes, might want to do it a bit better!
            if (target_z > 0.1 and state[2] < 0.02) or (target_z > 0.0 and state[2] < 0.0005):
                ret = -100
            altitude_error = abs(current_target[2] - state[2])
            #penalise being too far in z!
            ret -= 0.5*altitude_error
            ret += max(3.0*self.waypoints.progress_to_next_target, -3.0)
            if self.waypoints.distance_to_next_target > 0:
                ret += 0.1 / self.waypoints.distance_to_next_target

        #YAW RATE IS WZ
        yaw_rate = abs(state[15])
        yaw_rate_penalty = 0.01 * yaw_rate**2  # Add penalty for high yaw rate
        ret -= yaw_rate_penalty
        
        roll = state[7]
        pitch = state[8]
        ret -= 0.005 * (roll**2 + pitch**2)
        
        #target raeched reward!
        if self.waypoints.target_reached:
            ret = 100
            # self.waypoints.advance_targets()
            self.waypoints.mark_target_for_advance()
            self._update_target_visuals()
            # if target_z < 0.1:
            #     self._just_advanced_target = 0
            
            
        if self.waypoints.all_targets_reached:
            ret += 200 * self.waypoints.num_targets
        return ret
        
        
    def _computeTerminated(self):
        if self._just_advanced_target > 0:
            # Skip termination checks this step
            print("GRACE PERIOD")
            self._just_advanced_target -= 1
            return False
        
        state = self._getDroneStateVector(0)
        #if we crash terminate
        current_target = self.waypoints.current_target
        # print(f"comp term, current target is: {current_target}")
        if current_target is not None:
            target_z = current_target[2]
            if (target_z > 0.1 and state[2] < 0.01) or (target_z > 0.0 and state[2] < 0.0005):
                print(f"[DEBUG] Current index: {self.waypoints.num_targets_reached}, Target: {self.waypoints.current_target}")
                print(f"episode terminated because of crashing, z is {state[2]}, and target z is {target_z}")
                return True
        #if we go out of flying area
        if np.linalg.norm(state[0:3]) > self.FLIGHT_DOME_SIZE:
            print(f"episode terminated because we went out of flying area, distance from origin is {np.linalg.norm(state[0:3])}")
            return True
        return False
    
    def _computeTruncated(self):
        # state = self._getDroneStateVector(0)
        if self.waypoints.all_targets_reached:
            print("truncate, raeched all targets")
            return True
        if self.step_counter/self.PYB_FREQ > self.EPISODE_LEN_SEC:
            print("truncate, exceeded eps time")
            return True
        return False
    
    def _computeInfo(self):
        """Compute additional info dictionary."""
        return {
            "targets_reached": self.waypoints.num_targets_reached,
            "targets_remaining": len(self.waypoints.targets),
            "distance_to_target": self.waypoints.distance_to_next_target,
            "all_targets_reached": self.waypoints.all_targets_reached
        }
        
    def close(self):
        """Clean up resources when closing."""
        self._clear_target_visuals()
        super().close()
        
    def step(self, action):
        self.waypoints.advance_if_needed()  
        return super().step(action)

    