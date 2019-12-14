import os
import pdb
from pybulletgym.envs.roboschool.robots.robot_bases import URDFBasedRobot
import numpy as np

class PendulumNormal(URDFBasedRobot):
    def __init__(self):
        URDFBasedRobot.__init__(self, 
                                model_urdf='../../../../urdf/normal/pendulum_normal.urdf',
                                robot_name='pendulum_normal',
                                action_dim=1,
                                obs_dim=2)

    def robot_specific_reset(self, bullet_client):
        self._p = bullet_client

        # # Remove default linear and angular damping to match raisim
        # for part_id, part in self.parts.items():
        #     self._p.changeDynamics(part.bodyIndex, part.bodyPartIndex, lateralFriction=0.0, spinningFriction=0.0, rollingFriction=0.0, linearDamping=0.0, angularDamping=0.0, maxJointVelocity=1000)

        self.j1 = self.jdict["theta"]
        self.j1.reset_current_position(np.pi*np.random.uniform(-1,1), 0.01*np.random.uniform(-1,1))
        self.j1.set_motor_torque(0)

    def apply_action(self, a):
        assert( np.isfinite(a).all() )
        if not np.isfinite(a).all():
            print("a is inf")
            a[0] = 0
        self.j1.set_motor_torque(a[0])

    def calc_state(self):
        self.theta, self.theta_dot = self.j1.current_position()

        if not np.isfinite(self.theta):
            print("theta is inf")
            self.theta = 0

        return np.array([
            self.theta, self.theta_dot
        ])