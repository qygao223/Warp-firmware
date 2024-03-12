## Author
Name: Qianya Gao(Helen) College: Trinity Hall CRSid: qg223

## Overview
Using the sample data collected from the register of z axis of the MMA8451Q sensor, the front or back facing orientation of the board can be determined. An decision line of value = 0 is drawn based on the assumption that the collected sample data are in Gaussian distributions with means about +-4200. If mean of test data is larger than zero, the orientation is judged as front facing and vice versa. Since the data are very precise and means are far from decision line, the inaccuracy of this algorithm is approximately zero

## Boot.c
MMA8451Q relate to option 'y' from line 2715

## devMMA8451Q.h
header file for devMMA8451Q.c that declare functions

## devMMA8451Q.c
Contain function to read and write register(given), writting 0x05 to 0x2A to activate reading. Decision function called decideorientation(), detailed explaination see comments in the function from line 484 in .c file

## run code
By JLink RTT viewer, select 'a' then '5' to select sensor MMA8451Q. Then select 'y' to read MMA8451Q data. If then choose '0' for hexflag, first line shows data in register ox11, 10, 2a. Following are 100 data for accerelation in x,y,z axis in order with 50ms delay interval. If then choose '1', 100 test data from z axis will be collected and if the board is in front(+) or back(-) direction is decided by the sign of data's mean