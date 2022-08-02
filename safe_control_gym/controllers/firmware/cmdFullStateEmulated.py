#!/usr/bin/env python
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

import safe_control_gym.controllers.firmware.firmware_wrapper as em
import uav_trajectory


def pwm_to_g(pwm):
    '''
    for all four motors, this returns g that the crazyflie lifts. 27g total is hovering 
    
    F = ma 
    ma = Fg + Ft + Fn 


    0 = -9.81 * (m + m_cf) + Ft + Fn 
    Fn = 9.81 * (m + m_cf) - Ft 

    Scale = Fn / 9.81
    Ft = 9.81 * (m + m_cf - scale)
    Ft = -9.81*scale
    '''
    
    return 9.81*(0.409e-3*(pwm / 65536 * 256)**2 + 140.5e-3*(pwm / 65536 * 256) - 0.099) / 4

if __name__ == "__main__":
    trajpath = "figure8.csv"
    traj = uav_trajectory.Trajectory()
    traj.loadcsv(trajpath)
    # print("DUR", traj.duration)
    # raise

    sim_freq = 1000
    dt = 1 / sim_freq
    steps = int(sim_freq * traj.duration)

    rpy = np.array([0.0, 0.0, 0.0])
    _rpy = np.array([0.0, 0.0, 0.0])
    pos = np.array([0.0, 0.0, 0.0])
    vel = np.array([0.0, 0.0, 0.0])
    acc = np.array([0.0, 0.0, 0.0])

    emulator = em.PIDEmulator()
    com_to_motor = 56.65 / 2000 # m
    I = np.array([
        [.023951, 0.0, 0.0],
        [0.0, .023951, 0.0],
        [0.0, 0.0, .032347]
    ]) # g * m^2
    I_inv = np.linalg.inv(I)

    s = time.time()
    res = []
    for i in range(steps):
        e = traj.eval(i * dt)

        if i > 0:
            thrusts = np.array([pwm_to_g(pwm) for pwm in emulator.pwms])
            t = np.sum(thrusts)
            norm = R.from_euler('xyz', rpy).as_matrix()[:,2]

            dacc = t / 27 * norm + np.array([0, 0, -9.81])
            dvel = acc*dt
            dpos = vel*dt
            
            tau = np.array([0.0, 0.0, 0.0])
            tau[0] = (np.mean(thrusts[2:4]) - np.mean(thrusts[0:3])) * com_to_motor
            tau[1] = (np.mean(thrusts[1:3]) - np.mean(thrusts[[0,3]])) * com_to_motor
            # tau[2] = (thrusts[0] + thrusts[2] - thrusts[1] - thrusts[3]) / 1000)

            alpha = I_inv @ tau 
            # print(tau, thrusts)
            
            drpy = _rpy*dt
            d_rpy = alpha*dt

            pos += dpos
            vel += dvel
            acc = dacc
            # rpy += _rpy
            # _rpy += d_rpy

            # if i > 10: 
            #     break
            res += [emulator.state.position.z]

        '''
        Current state of this script: 
        - demonstrated to work as expected for 1D case 
        - rpy estimation never worked out, so anything not 1D crashes hard. Ignoring for now as pybullet engine does all this for me 
        
        '''

        emulator._update_state(
            rpy = rpy.tolist(),
            pos = pos.tolist(),
            vel = vel.tolist(),
            # acc = acc.tolist()
        )
        emulator._update_sensorData(
            timestamp=i,
            acc_vals = [0, 0, 0],
            gyro_vals = [0, 0, 0],
            baro_vals = [10, 273],
        )
        emulator.sendFullStateCmd(
            pos = [0, 0, 1], #e.pos.tolist(),
            vel = [0, 0, 0], #e.vel.tolist(),
            acc = [0, 0, 0], #e.acc.tolist(),
            rpy = [0, 0, 0], #[0, 0, e.yaw],
            rpy_rate = [0, 0, 0],#e.omega.tolist(),
            timestep = i
        )
        # emulator.sendFullStateCmd(
        #     pos = e.pos.tolist(),
        #     vel = e.vel.tolist(),
        #     acc = e.acc.tolist(),
        #     rpy = [0, 0, e.yaw],
        #     rpy_rate = e.omega.tolist(),
        #     timestep = i
        # )

        emulator.step_controller()

        # --- test cmdFullState     (works)    (check the cmdFullState.py)
        # pos = [1.5, 0.0, HEIGHT]    # Meters
        # vel = [0.0, 0.0, 0.0]       # Meters / second
        # acc = [0.0, 0.0, 0.0]       # Meters / second^2
        # yaw = 3.14                  # Yaw angle. Radians.
        # omega = [0.0, 0.0, 0.0]     # Angular velocity in body frame. Radians / sec.
        # cf.cmdFullState(pos, vel, acc, yaw, omega)

        # roll pitch yaw thrust 
        print(emulator)
        # time.sleep(max(0, (s + i * dt) - time.time())) # Executes in real time 
    print((time.time() - s) / steps, "s / control loop")

    x = list(range(len(res)))
    plt.plot(x, res)
    plt.show()