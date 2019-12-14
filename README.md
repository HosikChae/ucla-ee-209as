# ucla-ee-209as
This repository is for hosting source files that can be used for reproducing the results for the project done in UCLA course EE 209AS: Computational Robotics. Because some packages are not re-distributable, this repository does not function as a standalone. Other packages must be installed by the user. This repository does provide:

- Pre-trained network weights used for transfer `/weights`

- URDF files `/urdf`

- PyBullet Gym environments `/gym_envs`

- Interface from RL algorithm to PyBullet Gym environment `PyBulletGymEnv.py`

- Tensorboard logs `/logs`

- Reward plots `/plots`


## How to Run

### 1. Install Packages
#### PyTorch
```shell
git clone https://www.github.com/aminsung/pytorch
cd pytorch
pip install numpy ninja pyyaml mkl mkl-include setuptools cmake cffi typing
git submodule sync
git submodule update --init --recursive
export USE_CUDA=1 USE_CUDNN=1 USE_MKLDNN=1
python setup.py install
```

#### TensorboardX
```shell
pip install tensorboardX
```

#### PyBullet
```shell
pip install pybullet
```

#### OpenAI Gym
```shell
git clone https://github.com/openai/gym.git
cd gym
pip install -e .
```

#### PyBullet Gym
```shell
git clone https://github.com/benelot/pybullet-gym.git
cd pybullet-gym
pip install -e .
```

### 2. Download Necessary URDFs, Weights
#### Clone Repository
```shell
git clone https://github.com/aminsung/ucla-ee-209as
```

#### Install Custom Pendulum & Cart Pole
```shell
cd ucla-ee-209as
cp -a gym_envs/envs /path-to-pybullet-gym-repo/pybulletgym/envs
cd /path-to-pybullet-gym-repo
pip install -e .
```

#### Choose environment
Choose the model by changing the file name in `gym_envs/envs/roboschool/robots/pendula/pendulum_normal.py` and `gym_envs/envs/roboschool/robots/pendula/pendulum_cart.py`

### 3. SAC
#### Run SAC with the following parameters
```
gamma: 0.99
hidden size: 256
layer: 2
batch size: 256
actor learning rate: 3e-4
critic learning rate: 3e-4
alpha learning rate: 3e-4
tau: 0.005
target update interval: 1
```
Note that the weights are provided only in PyTorch.

#### Interface with PyBullet Gym
```python
from src.env.PyBulletGymEnv import PyBulletGymEnv as EnvironmentPyBullet

pendulum_type = 'normal' # normal || cart
env = EnvironmentPyBullet(pendulum_type, cfg['environment'])

obs = env.reset()
# get action from your network
obs, reward, done, infos = env.step(action, visualze=False) # turn visualize to true to render image
obs, infos = self.env.reset_and_update_info()
```

## URDF Explanation
There are two sub-folders (normal, cart) containing the different URDF files.

`pendulum_normal/cart`: Initial model with no joint and inertial uncertainties.

`pendulum_normal/cart_01`: Initial model with joint damping and no inertial uncertainties.

`pendulum_normal/cart_02`: Initial model with joint friction and no inertial uncertainties.

`pendulum_normal/cart_03`: Initial model with no joint uncertainties but with inertial uncertainties.

`pendulum_normal/cart_04`: Initial model with joint damping and inertial uncertainties.

`pendulum_normal/cart_05`: Initial model with joint friction and inertial uncertainties.

## Weights Explanation
There are two sub-folders (normal, cart) containing the different weights for the actor, critic, and target critic networks.

`van`: Agent trained with no domain randomization.

`sdr`: Agent trained with "small" domain randomization.

`bdr`: Agent trained with "big" domain randomization.

`edr`: Agent trained using EDR. (Only available in `cart`)

## Visualize Logs
```shell
tensorboard --logdir logs
```