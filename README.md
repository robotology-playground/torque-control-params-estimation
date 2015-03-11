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
4. Run `controlBoardDumper` to receive information about Voltage (PWM) and Current **OPTIONAL**
5. Run `matlab` 

## In Matlab
- Open and set the script `dump_joints.m` to load on matlab environment all variables and read all information
- Run the script `dump_joints.m` to load on matlab environment all variables and read all information
