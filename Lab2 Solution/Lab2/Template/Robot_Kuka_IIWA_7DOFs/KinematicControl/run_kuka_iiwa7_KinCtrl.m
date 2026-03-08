% Note: The first time you run this script, it may take some time since it
% has to load the simulink environment (usually a couple of seconds).
clear all
restoredefaultpath

% Adds the additional folders with functions into the library path
addpath('../DrawCFs')
addpath('../Model')
addpath('../Control')

% Defines the robot variables needed for the simulation
robot_name="kukaIiwa7"; 
n_dofs=7;

% Opens the simulator. Press play to run the simulation
open('KSimulator_kuka_iiwa7_kinCtrl')
