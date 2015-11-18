# FrictionJoint
This simulonk model allows one to identify viscous and coulomb friction of the iCub joints. 

## Dependencies
The model depends on *yarp*, *icub-main*, *codyco-superbuild* and *WBI-toolbox*, see the instructions to install them:
- [ICub Software Installation](http://wiki.icub.org/wiki/ICub_Software_Installation)
- [Codyco superbuild](https://github.com/robotology/codyco-superbuild)
- [WBI-Toolbox](https://github.com/robotology-playground/WBI-Toolbox)

## Installation
Download this repository
```
git clone https://github.com:robotology-playground/torque-control-params-estimation.git
```
## Initialization setup
Before launching any script related to this repository, it is important to configure the setup as follows:

1. Plug the computer, from which you want to launch the simulink model, into icub local area network
2. Run `robotInterface` on **pc104**
3. Run `wholeBodyDymanicsTree`;
4. Run `controlBoardDumper` to read PWMs and encoder speeds. For instance, if you want to estimate the friction of the `left_leg` joints you can launch the controlBoradDumper as follows:
```
controlBoardDumper --robot icub --part right_leg --rate 10  --joints "(0 1 2 3 4 5)" --dataToDump "(getOutputs getMotorEncoderSpeeds)"
controlBoardDumper --robot icub --part left_leg --rate 10  --joints "(0 1 2 3 4 5)" --dataToDump "(getOutputs getMotorEncoderSpeeds)"
controlBoardDumper --robot icub --part right_arm --rate 10  --joints "(0 1 2 3)" --dataToDump "(getOutputs getMotorEncoderSpeeds)"
controlBoardDumper --robot icub --part left_arm --rate 10  --joints "(0 1 2 3)" --dataToDump "(getOutputs getMotorEncoderSpeeds)"
controlBoardDumper --robot icub --part torso --rate 10  --joints "(0 1 2)" --dataToDump "(getOutputs getMotorEncoderSpeeds)"
```
5. Run `matlab` 
6. Open and set the script `dump_joints.m` to load on matlab environment all variables and read all information regarding your robot. 

## Perform joint friction estimation for a joint actuated by a single motor
- In the `dump_joints.m` file, set in the robot.joints attribute the joint that you want to estimate:
  - For example: `robot.joints = [robot.getJoint('l_hip_roll')];` .
  - Note that the joint names used for the iCub robot are the one documented in http://wiki.icub.org/wiki/ICub_joints .
- Run the script `dump_joints.m` to load in the matlab environment all variables and read all information and launch the simulink model used to dump data. 

##### Phase 1 - Friction Estimation
- At this point, switch the control mode of the select button "PID" for joint in `yarpmotorgui`. In "PID configuration" window write `0` in `Desired PWM max`.
- Run (pressing the play button) the simulink model for dumping the data, and then move the idle joint to acquire information about the friction. 
  - Just apply external force on the end effector (hands, foots). 
  - Do not reach the hardware joint limits, the force exerted by this mechanical constraint would corrupt the friction estimation.
  -  Try not to exert too much force as to not saturate the FT sensors, nor move the links too fast. 
- After you finished the idle data collection, stop the simulink model (or wait for the Simulink simulation to finish).
- Click on the "Save PWM=0 Data" button.

##### Phase 2 - Motor Torque Coefficient Estimation
- Switch the joint back to position control mode, return to the `yarpmotorgui` and click on "idle". Then, click on the "PID" button and in the "Position PID" tab write the previous value for `Desired PWM max`. Finally, click on "Send" and select "run" on `yarpmotorgui`.
- Apply again external force on the end effector to exert a torque on the joint. Being controlled in position, the joint now will not move. This is necessary to estimate the model between the motor input and the exerted torque. 
- After you finished the data collection, stop the simulink model (or wait for the Simulink simulation to finish).
- Click on the "Save Ref Data" button. 
- Click on "Plot Joint" and you will get the plot of the experimental data and the fitted model. 
- All the data is also saved in the directory that you indicated in the constructor of the Robot object. 

## Perform joint friction estimation for coupled joints
The procedure is similar to the one done for a single joint, the main differences are:
 - In the `dump_joints.m` file you have to set the `robot.joints` attribute using a special function to add all the joints belonging to the coupled group, the one currently supported are: 
    - `robot.joints = [robot.getCoupledJoints('torso')];` ,
    - `robot.joints = [robot.getCoupledJoints('l_shoulder')];` ,
    - `robot.joints = [robot.getCoupledJoints('r_shoulder')];` ,
 - Then, when you do the data collection you should write `0` in `Desired PWM max` for all the joints in the coupled group and collect the "PWM=0" data. Similarly after you click  on the "Save PWM=0 Data" button, you have to switch all the joints in the coupled group to be controlled in position, and you have to collect the motor gain data and then click on the "Save Ref Data" button. Make sure you "excite" every joint involved in the coupled group!
