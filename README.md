# NCND - Control of a 3D Quadrotor #

In this project implemented quad rotor flight controller in C++.
Code is in the /cpp directory.

`/cpp/config/QuadControlParams.txt` : contains parameters for controller.
`/cpp/src/QuadControl.cpp` : implemented flight controller code.

### Intro (scenario 1) ###

Adjusted the mass of drone in `QuadControlParams.txt` Mass = 0.5.

### Body rate and roll/pitch control (scenario 2) ###

#### 1. implement the code in the function `GenerateMotorCommands()` ####

In this code convert a desired 3-axis moment and collective thrust command to individual motor thrust commands.


f1, f2, f3, f4 are individual motors thrust, located front left, front right, rear left and rear right.

kappa is drag / thrust ratio and l is drone arm length.

total thrust = f1 + f2 + f3 + f4

Roll is produced by 1st and 3rd propellers - 2nd and 4th propellers.
tau_x = (f1 + f3 - f2 - f4 ) / l

Pitch is produced by 1st and 2nd propellers - 3rd and 4th propellers.
tau_y = (f1 + f2 - f3- f4) / l

Yaw is produced by clockwise counterclockwise propeller difference.
tau_z = -1 * (f1 - f2 + f4 - f3) * k

#### 2. implement the code in the function `BodyRateControl()`####

Implemented P controller to calculate a desired 3-axis moment given a desired and current body rate.

#### 3. Tune `kpPQR` in `QuadControlParams.txt` to get the vehicle to stop spinning quickly but not overshoot ####

Adjust the kpPQR parameter to stop drone flipping.

#### 4. Implement roll / pitch control ####

Implemented P controller to calculate a desired pitch and roll angle rates based on a desired global lateral acceleration, the current attitude of the quad, and desired collective thrust command


#### 5. Tune `kpBank` in `QuadControlParams.txt` to minimize settling time but avoid too much overshoot ####

Adjust the kpBank param to quad level itself, though it’ll still be flying away slowly since we’re not controlling velocity/position

<p align="center">
<img src="animations/scenario2.gif" width="500"/>
</p>


### Position/velocity and yaw angle control (scenario 3) ###

Implemented the position, altitude and yaw control for quad.  For the simulation use `Scenario 3`.  This create 2 identical quads, one offset from its target point (but initialized with yaw = 0) and second offset from target point but yaw = 45 degrees.

#### 1. implement the code in the function `LateralPositionControl()` ####

Implemented PID controller to calculate a desired horizontal acceleration based on desired lateral position/velocity/acceleration and current pose


#### 2. implement the code in the function `AltitudeControl()` ####

Implemented PID controller to calculate desired quad thrust based on altitude setpoint, actual altitude,
 vertical velocity setpoint, actual vertical velocity, and a vertical
acceleration feed-forward command

#### 3. tune parameters

Adjust `kpPosZ`, `kpPosZ`,  `kpVelXY` and `kpVelZ` parameters to move drones to target points.

#### 5. implement the code in the function `YawControl()` ####

Implemented P controller to calculate a desired yaw rate to control yaw to yawCmd.

#### 6. tune parameters `kpYaw` and the 3rd (z) component of `kpPQR` ####

<p align="center">
<img src="animations/scenario3.gif" width="500"/>
</p>

### Non-idealities and robustness (scenario 4) ###

In this part, we will explore some of the non-idealities and robustness of a controller.  For this simulation, we will use `Scenario 4`.  This is a configuration with 3 quads that are all are trying to move one meter forward.  However, this time, these quads are all a bit different:
 - The green quad has its center of mass shifted back
 - The orange vehicle is an ideal quad
 - The red vehicle is heavier than usual

Tuned the integral control, and other control parameters until all the quads successfully move properly.

<p align="center">
<img src="animations/scenario4.gif" width="500"/>
</p>


### Tracking trajectories (scenario 5) ###

Tested it's performance on a trajectory. use `Scenario 5`.  This scenario has two quadcopters:

 - the orange one is following `traj/FigureEight.txt`
 - the other one is following `traj/FigureEightFF.txt` - for now this is the same trajectory.

 Tuned parameters to pass the test. Red one much harder to adjust parameter..
 
