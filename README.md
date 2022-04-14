# 1 step autonomous multi-robot control
Uses a modification of the boids flocking rules in a system of multiple robots in a leader-follower structure. 

## What are boids rules ?

# How to Use
Prerequisites: Requires Python3, pygame, numpy

Run the main file and use the following scheme to control the leader:

"W", "S" : Increase and decrease linear velocity respectively

"A", "D" : Increase angular velocity anticlockwise and clockwise respectively

<img width="936" alt="1 step robot control" src="https://user-images.githubusercontent.com/95622570/163323944-78a3668d-a629-4f12-b9f6-6da77b48959b.png">

Green lines repersent exchange of state information (position velocity). 

Red lines represent direction of velocity. 

Blue lines represent the avoidance method being active due to close proximity to other robot(s)

Notes: Performance depends on efficient control of leader as this does not use any future prediction optimization and therefore the limited motion of the robots causes them to collide if they are clustered together.

Future work may include methods like MPC (Model Predictive Control) where the future trajectories can be computed and adjusted accordingly.  
