#!/usr/bin/env python
import time
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R

import safe_control_gym.controllers.firmware.firmware_wrapper as em


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
    sim_freq = 1000
    dt = 1 / sim_freq
    steps = int(sim_freq * 10)
    
    rpy = np.array([0.0, 0.0, 0.0])
    pos = np.array([0.0, 0.0, 1.0])
    vel = np.array([0.0, 0.0, 0.0])
    acc = np.array([0.0, 0.0, 0.0])

    emulator = em.PIDEmulator()
    emulator.sendFullStateCmd(pos, vel, acc, rpy, [0, 0, 0], 0)

    s = time.time()
    heights = []
    pwms = []
    for i in range(steps):
        if i > 0:
            # Simple 1D physics, no rpy  
            thrusts = np.array([pwm_to_g(pwm) for pwm in emulator.pwms])
            t = np.sum(thrusts)
            norm = R.from_euler('xyz', rpy).as_matrix()[:,2]

            dacc = t / 27 * norm + np.array([0, 0, -9.81])
            dvel = acc*dt
            dpos = vel*dt

            pos += dpos
            vel += dvel
            acc = dacc

            heights += [emulator.state.position.z]
            pwms += [emulator.pwms]

        # if i == 3000:
        #     emulator.sendTakeoffVelCmd(3, 1, False)

        emulator._update_state(
            rpy = rpy,
            pos = pos.tolist(),
            vel = vel.tolist(),
        )

        emulator._update_sensorData(
            timestamp=i,
            acc_vals = (acc - np.array([0, 0, -9.81])) / 9.81,
            gyro_vals = [0, 0, 0],
            baro_vals = [10, 273],
        )

        emulator.updateSetpoint(i*dt)
        
        emulator.step_controller()
        
        print(emulator)
        # time.sleep(max(0, (s + i * dt) - time.time())) # Executes in real time 

    print((time.time() - s) / steps, "s / control loop")

    x = list(range(len(heights)))

    fig = plt.figure(figsize=(16,8))
    ax = fig.add_subplot(121)
    ax.plot(x, heights)
    ax2 = fig.add_subplot(122)
    ax2.plot(x, np.array(pwms)[:,0])
    ax2.plot(x, np.array(pwms)[:,1])
    ax2.plot(x, np.array(pwms)[:,2])
    ax2.plot(x, np.array(pwms)[:,3])
    plt.show()