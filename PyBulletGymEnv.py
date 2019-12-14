import pdb
import time
import gym
import numpy as np
import pybulletgym.envs

class PyBulletGymEnv(object):
    def __init__(self, environment_type, cfg):
        """
        Return the corresponding PyBullet environment.
        :param string environment_type: Environment type
        :param dict cfg: Configuration file
        """
        env_type = "CustomPendulum"+environment_type.capitalize()+"PyBulletEnv-v0"
        self.env = gym.make(env_type)

        self.observation_space = self.env.observation_space
        self.action_space = self.env.action_space
        self.rewards = [[] for _ in range(1)]

        self.env.render()

    def step(self, action, visualize=False):
        # if not visualize:
        #     obs, rewards, done, extra_info = self.env.step(action.numpy())
        # else:
        #     obs, rewards, done, extra_info = self.env.test_step(action.numpy())
        obs, rewards, done, extra_info = self.env.step(action.numpy())

        if obs.ndim == 1:
            obs = np.array([obs])

        self.rewards[0].append(rewards)
        if type(rewards) != np.ndarray:
            rewards = np.array([rewards])

        if type(done) != np.ndarray:
            done = np.array([done])

        return obs, rewards, done, extra_info


    def reset(self):
        ret_env = self.env.reset()
        if ret_env.ndim == 1:
            ret_env = np.array([ret_env])
        return ret_env

    def reset_and_update_info(self):
        return self.reset(), self._update_epi_info()

    def _update_epi_info(self):
        info = [{} for _ in range(1)]
        eprew = sum(self.rewards[0])
        eplen = len(self.rewards[0])
        epinfo = {"r": eprew, "l": eplen}
        info[0]['episode'] = epinfo
        self.rewards[0].clear()

        return info

    def start_recording_video(self, file_name):
        self.env.start_recording_video(file_name)

    def stop_recording_video(self):
        self.env.stop_recording_video()