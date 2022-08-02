"""A quadrotor trajectory tracking example.

Notes:
    Includes and uses PID control.

Run as:

    $ python3 tracking.py --overrides ./tracking.yaml

"""
import time
import pybullet as p
from functools import partial
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as R

from safe_control_gym.utils.configuration import ConfigFactory
from safe_control_gym.utils.registration import make
from dataloader import CFDataLoader

def run(gui=None, max_steps=None):
    """The main function creating, running, and closing an environment.

    """

    # Create an environment
    CONFIG_FACTORY = ConfigFactory()               
    config = CONFIG_FACTORY.merge()
    
    # Set iterations and episode counter.
    # ITERATIONS = int(config.quadrotor_config['episode_len_sec']*config.quadrotor_config['ctrl_freq'])
    
    # Use function arguments for workflow testing
    if gui is not None:
        config.quadrotor_config['gui'] = gui


    dataloader = CFDataLoader('/home/spencer/Documents/DSL/CF Data/droneTests_08_10_2018/smallCircle1006')

    for i, command_series in enumerate(dataloader.commands):
        ITERATIONS = int((command_series[-1][0] - command_series[0][0]) * config.quadrotor_config['ctrl_freq'])
        if max_steps is not None:
            ITERATIONS = min(ITERATIONS, max_steps)
        # Start a timer.
        START = time.time()
                
        # Create controller
        env_func = partial(make,
                           'quadrotor',
                           **config.quadrotor_config
                           )
        ctrl = make('firmware',
                    env_func,
                    ) 
                    
        # Load trajectory 
        traj = {}
        traj[0] = command_series[0][1]
        start_idx = 0
        for j in range(ITERATIONS):
            _time = j*ctrl.dt
            idx = get_closest(command_series, _time, start_idx)
            if idx is not None:
                traj[j] = command_series[idx][1]
                start_idx = idx

        initial_pos = command_series[0][1]
        p.resetBasePositionAndOrientation(ctrl.env.DRONE_IDS[0],
                                    initial_pos[:-1],
                                    p.getQuaternionFromEuler([0, 0, initial_pos[-1]]),
                                    physicsClientId=ctrl.env.PYB_CLIENT)


        results = ctrl.run(ITERATIONS, traj) 

        actions = np.array(results['action'])
        timesteps = np.array(list(range(len(actions))))
        fig = plt.figure(figsize=(9, 9))
        ax = fig.add_subplot(111)
        ax.plot(timesteps, actions[:,0], label='FL')
        ax.plot(timesteps, actions[:,1], label='BL')
        ax.plot(timesteps, actions[:,2], label='BR')
        ax.plot(timesteps, actions[:,3], label='FR')
        ax.legend()

        plt.show()

        _gt_pos = dataloader.positions[i]
        gt_pos = []
        start_idx = 0
        for j in range(ITERATIONS):
            _time = j*ctrl.dt
            idx = get_closest(_gt_pos, _time, start_idx)
            if idx is not None:
                gt_pos += [_gt_pos[idx][1]]
                start_idx = idx
            else:
                gt_pos += [[None, None, None]]
        gt_pos = np.array(gt_pos)
        pos = np.array(results['obs'])[:,[0, 2, 4, 8]]
        timesteps = np.array(list(range(len(pos))))
        fig = plt.figure(figsize=(16, 9))
        ax1 = fig.add_subplot(131)
        ax1.plot(timesteps, pos[:,0], label='X')
        ax1.plot(timesteps, gt_pos[:,0], label='X_gt')
        ax1.legend()

        ax1 = fig.add_subplot(132)
        ax1.plot(timesteps, pos[:,1], label='Y')
        ax1.plot(timesteps, gt_pos[:,1], label='Y_gt')
        ax1.legend()

        ax1 = fig.add_subplot(133)
        ax1.plot(timesteps, pos[:,2], label='Z')
        ax1.plot(timesteps, gt_pos[:,2], label='Z_gt')
        ax1.legend()
        
        # ax.plot(timesteps, pos[:,1], label='Y')
        # ax.plot(timesteps, pos[:,2], label='Z')
        # ax.plot(timesteps, pos[:,1], label='Y')
        # ax.plot(timesteps, pos[:,2], label='Z')
        # ax.plot(timesteps, pos[:,3], label='yaw')
        

        plt.show()

        ctrl.close()            

        elapsed_sec = time.time() - START
        print("\n{:d} iterations (@{:d}Hz) and {:d} episodes in {:.2f} seconds, i.e. {:.2f} steps/sec for a {:.2f}x speedup.\n"
              .format(ITERATIONS, config.quadrotor_config.ctrl_freq, 1, elapsed_sec, ITERATIONS/elapsed_sec, (ITERATIONS*(1. / config.quadrotor_config.ctrl_freq))/elapsed_sec))


def get_closest(data, target, start_idx=0):
    min_dist = 0.01
    return_idx = None
    for i in range(start_idx, len(data)):

        if abs(data[i][0] - target) < min_dist:
            return_idx = i
            min_dist = abs(data[i][0] - target)
        elif return_idx is not None:
            break
    return return_idx

if __name__ == "__main__":
    run()