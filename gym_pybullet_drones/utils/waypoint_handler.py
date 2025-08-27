"""Handler for Waypoints in the environments."""

from __future__ import annotations

import math
import os

import numpy as np
import pybullet as p
from typing import Optional


class WaypointHandler:
    """Handler for Waypoints in the environments."""

    def __init__(
        self,
        # enable_render: bool,
        num_targets: int,
        use_yaw_targets: bool,
        goal_reach_distance: float,
        goal_reach_angle: float,
        flight_dome_size: float,
        min_height: float,
        np_random: np.random.Generator,
        client_id: int,
        random_mode: bool = True,
        waypoints: Optional[np.ndarray] = None,
        yaw_waypoints: Optional[np.ndarray] = None,
    ):
        """__init__.

        Args:
            enable_render (bool): enable_render
            num_targets (int): num_targets
            use_yaw_targets (bool): use_yaw_targets
            goal_reach_distance (float): goal_reach_distance
            goal_reach_angle (float): goal_reach_angle
            flight_dome_size (float): flight_dome_size
            min_height (float): min_height
            np_random (np.random.Generator): np_random

        """
        # constants
        # self.enable_render = enable_render
        self.num_targets = num_targets if random_mode else len(waypoints)
        self.use_yaw_targets = use_yaw_targets
        self.goal_reach_distance = goal_reach_distance
        self.goal_reach_angle = goal_reach_angle
        self.flight_dome_size = flight_dome_size
        self.min_height = min_height
        self.np_random = np_random
        ##new bit#######################
        self.random_mode = random_mode
        self.custom_waypoints = waypoints
        self.custom_yaws = yaw_waypoints
        self.CLIENT_ID = client_id
        ################################
        
        # State variables
        self.targets = np.array([])
        self.yaw_targets = np.array([])
        self.new_distance = np.inf
        self.old_distance = np.inf
        self.yaw_error_scalar = 0.0
        self._should_advance = False


    def reset(
        self,
        np_random: None | np.random.Generator = None,
    ):
        """Resets the waypoints."""

        # update the random state
        if np_random is not None:
            self.np_random = np_random

        # reset the error
        self.new_distance = np.inf
        self.old_distance = np.inf

        if self.random_mode:
            # we sample from polar coordinates to generate linear targets
            self.targets = np.zeros(shape=(self.num_targets, 3))
            thetas = self.np_random.uniform(0.0, 2.0 * math.pi, size=(self.num_targets,))
            phis = self.np_random.uniform(0.0, 2.0 * math.pi, size=(self.num_targets,))
            for i, theta, phi in zip(range(self.num_targets), thetas, phis):
                dist = self.np_random.uniform(low=1.0, high=self.flight_dome_size * 0.8)
                x = dist * math.sin(phi) * math.cos(theta)
                y = dist * math.sin(phi) * math.sin(theta)
                z = abs(dist * math.cos(phi))

                # check for floor of z
                self.targets[i] = np.array(
                    [x, y, z if z > self.min_height else self.min_height]
                )
        else:
            if self.custom_waypoints is not None:
                self.targets = self.custom_waypoints.copy()
            else:
                raise ValueError("Custom waypoints not provided in non-random mode")

        # yaw targets
        if self.use_yaw_targets:
            if self.random_mode:
                self.yaw_targets = self.np_random.uniform(
                    low=-math.pi, high=math.pi, size=(self.num_targets,)
                )
            else:
                if self.custom_yaws is not None:
                    self.yaw_targets = self.custom_yaws.copy()
                else:
                    raise ValueError("Custom yaw targets not provided in non-random mode")

    @property
    def distance_to_next_target(self) -> float:
        """distance_to_next_target.

        Returns:
            float:
        """
        return self.new_distance

    def distance_to_targets(
        self,
        ang_pos: np.ndarray,
        lin_pos: np.ndarray,
        quaternion: np.ndarray,
    ):
        """distance_to_targets.

        Args:
            ang_pos (np.ndarray): ang_pos
            lin_pos (np.ndarray): lin_pos
            quaternion (np.ndarray): quaternion

        """
        if len(self.targets) == 0:
            return np.array([[0, 0, 0]])
        
        rotation = np.array(p.getMatrixFromQuaternion(quaternion, physicsClientId=self.CLIENT_ID)).reshape(3, 3)

        # drone to target
        target_deltas = np.matmul((self.targets - lin_pos), rotation)

        # record distance to the next target
        self.old_distance = self.new_distance
        self.new_distance = float(np.linalg.norm(target_deltas[0]))

        if self.use_yaw_targets:
            yaw_errors = self.yaw_targets - ang_pos[-1]

            # rollover yaw
            yaw_errors[yaw_errors > math.pi] -= 2.0 * math.pi
            yaw_errors[yaw_errors < -math.pi] += 2.0 * math.pi
            yaw_errors = yaw_errors[..., None]

            # add the yaw delta to the target deltas
            target_deltas = np.concatenate([target_deltas, yaw_errors], axis=-1)

            # compute the yaw error scalar
            self.yaw_error_scalar = np.abs(yaw_errors[0])

        return target_deltas

    @property
    def progress_to_next_target(self):
        """progress_to_target."""
        if np.any(np.isinf(self.old_distance + self.new_distance)):
            return 0.0
        return self.old_distance - self.new_distance

    @property
    def target_reached(self):
        """target_reached."""
        if not self.new_distance < self.goal_reach_distance:
            return False

        if not self.use_yaw_targets:
            return True

        if self.yaw_error_scalar < self.goal_reach_angle:
            return True

        return False

    def advance_targets(self):
        """advance_targets."""
        if len(self.targets) > 1:
            self.targets = self.targets[1:]
            if self.use_yaw_targets:
                self.yaw_targets = self.yaw_targets[1:]
        else:
            self.targets = np.array([]).reshape(0, 3)
            if self.use_yaw_targets:
                self.yaw_targets = np.array([])

    @property
    def num_targets_reached(self):
        """num_targets_reached."""
        return self.num_targets - len(self.targets)

    @property
    def all_targets_reached(self):
        """all_targets_reached."""
        return len(self.targets) == 0
    
    @property
    def current_target(self) -> Optional[np.ndarray]:
        """Get the current target position."""
        if len(self.targets) > 0:
            return self.targets[0]
        return None
    
    @property
    def current_yaw_target(self) -> Optional[float]:
        """Get the current yaw target."""
        if self.use_yaw_targets and len(self.yaw_targets) > 0:
            return self.yaw_targets[0]
        return None

    @property
    def remaining_targets(self) -> np.ndarray:
        """Get all remaining targets."""
        return self.targets.copy()
    
    @property
    def should_advance_target(self):
        return self._should_advance

    def mark_target_for_advance(self):
        self._should_advance = True

    def advance_if_needed(self):
        if self._should_advance:
            self.advance_targets()
            self._should_advance = False
