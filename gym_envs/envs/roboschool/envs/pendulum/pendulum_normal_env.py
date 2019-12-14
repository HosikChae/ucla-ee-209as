from pybulletgym.envs.roboschool.envs.env_bases import BaseBulletEnv
from pybulletgym.envs.roboschool.robots.pendula.pendulum_normal import PendulumNormal
from pybulletgym.envs.roboschool.scenes.scene_bases import SingleRobotEmptyScene
import pybullet as p
import pdb
import numpy as np
import cv2
import imageio


class PendulumNormalEnv(BaseBulletEnv):
    def __init__(self):
        self.robot = PendulumNormal()
        BaseBulletEnv.__init__(self, self.robot, render=False)
        self.stateId = -1

        self.recording = False

    def create_single_player_scene(self, bullet_client):
        return SingleRobotEmptyScene(bullet_client, gravity=9.8, timestep=0.001, frame_skip=50)

    def reset(self):
        # if self.stateId >= 0:
        #     self._p.restoreState(self.stateId)
        r = BaseBulletEnv._reset(self)
        # if self.stateId < 0:
        #     self.stateId = self._p.saveState()
        return r

    def step(self, a):
        self.camera_adjust()
        self.robot.apply_action(a)
        self.scene.global_step()
        state = self.robot.calc_state()
        reward = np.cos(self.robot.theta)
        done = False
        self.rewards = [float(reward)]
        self.HUD(state, a, done)

        self._record_video()        

        # Create infos
        infos = []
        infos.append({'extra_info': { 'alive': 1.0 }})
        infos.append({'extra_info': { 'x': np.sin(self.robot.theta) }})
        infos.append({'extra_info': { 'y': np.cos(self.robot.theta) }})
        infos.append({'extra_info': { 'theta dot': self.robot.theta_dot }})
        infos.append({'extra_info': { 'terminal': 0.0 }})

        return state, sum(self.rewards), done, infos

    def test_step(self, a):
        pass

    def camera_adjust(self):
        # pos, orn = self.robot._p.getBasePositionAndOrientation(1)
        # pos[2] = 2.5

        self.robot._p.resetDebugVisualizerCamera(1.5, 3.14, -1.3, (0, 0, 2.5))
        # self.camera.move_and_look_at(0, 1.2, 1.0, 0, 0, 0.5)

    def start_recording_video(self, vid_name):
        fps = int(1/(self.scene.timestep*self.scene.frame_skip))

        # self.log_id = self.robot._p.startStateLogging(self._p.STATE_LOGGING_VIDEO_MP4, vid_name)
        
        # codec = cv2.VideoWriter_fourcc(*'XVID')
        # self.vid = cv2.VideoWriter(vid_name, codec, fps, (1024, 768))

        self.vid = imageio.get_writer(vid_name, format='FFMPEG', fps=fps)

        print("Started recording in PyBullet!")
        self.recording = True

    def stop_recording_video(self):
        # self.robot._p.stopStateLogging(self.log_id)
        # self.vid.release()
        self.vid.close()
        print("Stopped recording in PyBullet!")
        self.recording = False

    def _record_video(self):
        if self.recording:
            img = self.robot._p.getCameraImage(width=1024, height=768, renderer=self.robot._p.ER_BULLET_HARDWARE_OPENGL)
            # self.vid.write(cv2.cvtColor(img[2], cv2.COLOR_RGBA2BGR))
            self.vid.append_data(cv2.cvtColor(img[2], cv2.COLOR_RGBA2RGB))