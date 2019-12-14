import os
import pdb
from pybulletgym.envs.roboschool.robots.robot_bases import URDFBasedRobot
import numpy as np

class PendulumCart(URDFBasedRobot):
    def __init__(self):
        URDFBasedRobot.__init__(self, 
                                model_urdf='../../../../urdf/cart/pendulum_cart.urdf',
                                robot_name='pendulum_cart',
                                action_dim=1,
                                obs_dim=5)

    def robot_specific_reset(self, bullet_client):
        self._p = bullet_client

        # # Remove default linear and angular damping to match raisim
        # for part_id, part in self.parts.items():
        #     self._p.changeDynamics(part.bodyIndex, part.bodyPartIndex, lateralFriction=0.0, spinningFriction=0.0, rollingFriction=0.0, linearDamping=0.0, angularDamping=0.0, maxJointVelocity=1000)

        self.j1 = self.jdict["slider_to_cart"]
        self.j1.reset_current_position(0.0, 0.0)
        self.j1.set_motor_torque(0)

        self.j2 = self.jdict["cart_to_arm"]
        self.j2.reset_current_position(np.pi*np.random.uniform(-1,1), 0.01*np.random.uniform(-1,1))
        # self.j2.reset_current_position(0.785, 0.0)
        self.j2.set_motor_torque(0)

    def apply_action(self, a):
        assert( np.isfinite(a).all() )
        if not np.isfinite(a).all():
            print("a is inf")
            a[0] = 0
        self.j1.set_motor_torque( 5.*np.clip(a[0], -1., 1.) )

    def calc_state(self):
        self.slider_pos, self.slider_vel = self.j1.current_position()
        self.theta, self.theta_dot = self.j2.current_position()

        if not np.isfinite(self.theta):
            print("theta is inf")
            self.theta = 0

        return np.array([
            self.slider_pos, self.slider_vel, np.sin(self.theta), np.cos(self.theta), self.theta_dot
        ])