# 1 step autonomous multi-robot control
Uses a modification of the boids flocking rules in a system of multiple robots in a leader-follower structure. 


# How to Use
Prerequisites: Requires Python3, pygame, numpy

Run the main file and use the following scheme to control the leader:

"W", "S" : Increase and decrease linear velocity respectively
"A", "D" : Increase angular velocity anticlockwise and clockwise respectively

Notes: Does not perform well as, this does not use any future prediction optimization and therefore the limited motion of the robots causes them to collide if they are clustered together.

Future work may include methods like MPC (Model Predictive Control) where the future trajectories can be computed and adjusted accordingly.  
