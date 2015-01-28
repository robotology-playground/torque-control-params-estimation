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
4. Run `controlBoardDumper` to receive information about Voltage (PWM) and Current
5. Run `matlab`

## On Matlab
- Run the script `WBLmotor.m` to load on matlab environment all variables
- Open *simunlink* to dump all information about experiment
  - Open `MotorIdent.slx` to dump measure for single joint
  - Open `MotorCoupledIdent.slx` to dump measure for coupled joints
- To see all plots about data dumped run `FrictionTest`

### Other information
In `scripts` folder you have other script to compare or analyze information about joint or single data.

