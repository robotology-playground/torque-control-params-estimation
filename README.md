# FrictionJoint
Analysis friction on iCub's joint with **MATLAB** and **WBI-Toolbox**

## Dataset
- **iCubGenova04** [![DOI](https://zenodo.org/badge/doi/10.5281/zenodo.14814.svg)](http://dx.doi.org/10.5281/zenodo.14814)

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

- In `codyco-superbuild/main/WBIToolbox/libraries/wbInterface/conf/wholeBodyInterfaceToolbox/wholeBodyInterfaceToolbox.ini`
set:
```
robot          icubGazeboSim
localName      simulink
worldRefFrame  root_link
robot_fixed    true
wbi_id_list    JOINT_FRICION
wbi_config_file yarpWholeBodyInterface.ini
```
Finally go on `codyco-superbuild/build/` and `make`

## On Matlab
- Run the script `example.m` to load on matlab environment all variables and read all information
- Open *simunlink* to dump all information about experiment
  - Open `MotorIdent.slx` to dump measure for single joint
  - Open `MotorCoupledIdent.slx` to dump measure for coupled joints
- To see all plots about data dumped run `FrictionTest`

### Other information
In `scripts` folder you have other script to compare or analyze information about joint or single data.
`codyco-superbuild/src/modules/wholeBodyDynamicsTree/app/robots/`**NAME_ROBOT**`/wholeBodyDynamicsTree.ini`

## OLD information
- In `codyco-superbuild/libraries/yarpWholeBodyInterface/app/robots/`**NAME_ROBOT**`/yarpWholeBodyInterface.ini`
Under **[WBI_ID_LISTS]** add:
```
JOINT_FRICTION = (NAME_JOINT)
```
example:
```
JOINT_FRICTION = (l_hip_pitch)
```
Quickly way: `codyco-superbuild/build/install/share/codyco/robots/`**NAME_ROBOT**`/yarpWholeBodyInterface.ini`
