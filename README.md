# FrictionJoint
This Simulink model identifies the viscous and coulomb friction of the iCub joints. 

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
- In the `dump_joints.m` script set the variables
  - `robotName` (e.g. `robotName = iCubGenova02` or `robotName = iCubHeidelberg01`).
  - `codycoSuperbuildDir` (e.g. `codycoSuperbuildDir = /usr/local/src/robot/codyco-superbuild`).
  - `outputDir` (e.g. `outputDir = 'experiments'`).
- In the `dump_joints.m` script, head to the section `Add motors to test` and uncomment the `robot.joints` attribute for the joints that you want to estimate:
  - For example: `robot.joints = [robot.getJoint('l_hip_roll')];` .
  - Note that the joint names used for the iCub robot are the one documented in http://wiki.icub.org/wiki/ICub_joints.
- Run the script `dump_joints.m` to load in the matlab environment all variables and read all information and launch the simulink model used to dump data. 

##### Phase 1 - Friction Estimation
- At this point, switch the control mode of the select button "PID" for joint in `yarpmotorgui`. In the "Position PID" tab write `0` in `PID Output Limit` (**Write down the original value somewhere, as you will need it later!**).
- Run (pressing the play button) the simulink model for dumping the data, and then move the idle joint to acquire information about the friction. 
  - Just apply external force on the end effector (hands, foots). 
  - Do not reach the hardware joint limits, the force exerted by this mechanical constraint would corrupt the friction estimation.
  -  Try not to exert too much force as to not saturate the FT sensors, nor move the links too fast. 
- After you finished the idle data collection, stop the simulink model (or wait for the Simulink simulation to finish).
- Click on the "Save PWM=0 Data" button.

##### Phase 2 - Motor Torque Coefficient Estimation
- Switch the joint back to position control mode, return to the `yarpmotorgui` and click on "idle". Then, click on the "PID" button and in the "Position PID" tab write the original value for `PID Output Limit`. Finally, click on "Send", closing also the window, followed by "run" on the `yarpmotorgui`.
- Now, back on the Simulink model, click on the 'run' button.
- Apply again external force on the end effector to exert a torque on the joint. Being controlled in position, the joint now will not move. This is necessary to estimate the model between the motor input and the exerted torque. The same considerations done in Phase 1 apply here as well.
- After you finished the data collection, stop the simulink model (or wait for the Simulink simulation to finish).
- Click on the "Save Ref Data" button. 
- Click on "Plot Joint" and you will get the plot of the experimental data and the fitted model. 
- All the data is saved in the directory specified in the first steps. 

## Perform joint friction estimation for coupled joints
Proceed as in the **Phase 1** and **Phase 2** except that coupled joints are added as follows:
- `robot.joints = [robot.getCoupledJoints('torso')];` ,
- `robot.joints = [robot.getCoupledJoints('l_shoulder')];` ,
- `robot.joints = [robot.getCoupledJoints('r_shoulder')];` ,

iCub's coupled joints (torso and shoulders) should be moved as independently as possible for the best estimation possible.
