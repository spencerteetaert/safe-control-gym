GOAL: emulate lines 287 - 303 in stabilizer.c

TODO: 
- determine which sitaw cases should be used (tumble, freefall, at rest)
- validate ppeline with real data 
- create trajectory interfacing 
- fix timing/tick situation 
- add localization ? 
- add other sitaw 
- 



simulation planned           | accounted for    | Description 
-----------------------------|------------------|-------------------------------------------------------------
no, provided gt by pybullet  | no               | estimator estimates current state using sensor readings 
yes                          | yes              | commander recieves command from ground station. Sets setpoint 
yes                          | no               | returns CF to hover state if command timestep is too far from current timestep 
yes                          | yes              | runs trajectory calculations on baord if enabled (default yes) 
yes                          | TU is, AR/FF not | sitaw updates setpoint based off current state and sensor feedback
no, single drone only        | no               | collision avoidance runs adjusting setpoint based off other cf locations 
yes                          | yes              | controller runs on setpoint 
yes                          | yes              | powerDistribution.c converts command to PWM 
no, pwm used directly in sim | no               | motors.c update physical values based off PWM


Communication pipeline
- packet is processed by low level scripts 
- crtp_commander.c::commanderCrtpCB() handles packet 
- crtp_commander_generic::crtpCommanderGenericDecodeSetpoint() routes packet based on command type 
- crtp_commander_generic::fullStateDecoder() creates setpoint_t object from fullStateCmd packet 
- commander.c::commanderSetSetpoint() sets setpoint in RTOS queue (timestamp added here)
- commander.c::commanderGetSetpoint() is called in stabilizer loop to get the next setpoint 


Things I have removed: 
- tilt compensation in controller_pid 
- support for brushless motors 
- imu mag reading 


Open Qs:
- do we want the wrapper to use state estimation? 
- 