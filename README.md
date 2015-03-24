# FrictionJoint
Analysis friction on iCub's joint with **MATLAB** and **WBI-Toolbox**

## Dependencies
This scripts running with *yarp*, *icub-main*, *codyco-superbuild* and *WBI-toolbox*, see the instruction to install all programs:
- [ICub Software Installation](http://wiki.icub.org/wiki/ICub_Software_Installation)
- [Codyco superbuild](https://github.com/robotology/codyco-superbuild)
- [WBI-Toolbox](https://github.com/robotology-playground/WBI-Toolbox)

## Installation
Download this repository
```
git clone https://github.com/rbonghi/FrictionJoint/
```
## Initialization setup
Before run scripts are important to:

1. Plug the PC in icub local area network
2. Run `robotInterface` on **pc104**
3. Run `wholeBodyDymanicsTree` to receive information about torque on joints
4. Run `controlBoardDumper` to receive information about Voltage (PWM) and Current **OPTIONAL**
  - For example, if you want to estimate the friction of the `left_leg` joints you can launch controlBoradDumper with the following command: `controlBoardDumper --robot icub --part left_leg --rate 10  --joints "(0 1 2 3 4 5)" --dataToDump "(getOutputs getCurrents)"`
5. Run `matlab` 
6. Open and set the script `dump_joints.m` to load on matlab environment all variables and read all information regarding your robot. 

## Perform joint friction estimation for a joint actuated by a single motor
- In the `dump_joints.m` file, set in the robot.joints attribute the joint that you want to estimate:
  - For example: `robot.joints = [robot.getJoint('l_hip_roll')];` .
  - Note that the joint names used for the iCub robot are the one documented in http://wiki.icub.org/wiki/ICub_joints .
- Run the script `dump_joints.m` to load in the matlab environment all variables and read all information and launch the simulink model used to dump data. 
- At this point, switch the control mode of the select joint to "Idle", for example using the `yarpmotorgui`. 
- Run (pressing the play button) the simulink model for dumping the data, and then move the idle joint to acquire information about the friction. 
  - Just apply external force on the end effector (hands, foots). 
  - Do not reach the hardware joint limits, the force excerted by this mechanical constraint would corrupt the friction estimation 
- After you finished the idle data collection, stop the simulink model (or wait for the Simulink simulation to finish).
- Click on the "Save Idle Data" button.
- Switch the joint back in position control mode, for example clicking "Run" on the `yarpmotorgui`.
- Apply again external force on the end effector to excert a torque on the joint. Being controlled in position, the joint now will not move. This is necessary to estimate the model between the motor input and the excerted torque. 
- After you finished the data collection, stop the simulink model (or wait for the Simulink simulation to finish).
- Click on the "Save Ref Data" button. 
- Click on "Plot Joint" and you will get the plot of the experiments data and the fitted model. 
- All the data is also saved in the directory that you indicated in the constructor of the Robot object. 

## Perform joint friction estimation for group of coupled joints
The procedure is similar to the one done for a single joint, the main differences are:
 - In the `dump_joints.m` file you have to set the `robot.joints` attribute using a special function to add all the joints belonging to the coupled group, the one currently supported are: 
    - `robot.joints = [robot.getCoupledJoints('torso')];` ,
    - `robot.joints = [robot.getCoupledJoints('l_shoulder')];` ,
    - `robot.joints = [robot.getCoupledJoints('r_shoulder')];` ,
 - Then, when you do the data collection you should switch in idle all the joints in the coupled group and collect the idle data. Similarly after you clicked  the "Save Idle Data" button, you have to switch all the joints in the coupled group to be controlled in positions, and you have to collect the motor gain data and then click on the "Save Ref Data" button. Remember to try to get exciting data for all the motors belonging to the coupled group. 
