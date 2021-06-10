input file:
This stores all the files initially required for the scripts present. This includes the blf and rosbag files for the tests and a dbc file for OXTS.

mat files:
This folder stores all the mat files that are saved and loaded by all the scripts.

FOR VISUALIZATION:
birdseyeplot.m is the script that visualizes data, but before tha you have to run a couple of other scripts.
for visualization you need to run all the scipts in this folder except for sf_sep_var.m and range_plot.m
The order of running scripts is step1>step2>step3>step4 and so one for both sf and oxts.
(oxts_step3 has to be run after sf_step2)

FOR GRAPHING RANGE VS TIME AND RANGE RATE VS TIME:
for graphing you need to run the following scripts
sf_step1 > sf_step2
oxts_step1 > oxts_step2 > oxts_step3 (oxts_step3 has to be run after sf_step2)

since this scripts plots two different tests you will have to run the above scripts twice with different inputs and outputs.(once for the 10mph test and the second for the 20mph test)

before running the above scripts again, the following changes have to be made...
oxts_step1: the blf(line 16)
oxts_step3: the name of the ouput file(line 78)
sf_step1: the bag file(line 3)
sf_step2: the name of the ouput file(line 36)

finally run range_plot.m