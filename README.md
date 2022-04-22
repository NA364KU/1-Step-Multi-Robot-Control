# 1 Step Multi-Robot Control
Uses a modification of the boids flocking rules in a system of multiple robots in a leader-follower structure. 

## What are boids rules ?
Boids refers to a computer simulation which tries to mimic natural flocking behavior such as birds or fish. This is done by use of 3 rules known as "cohesion", "alignment", and "separation". 

**Cohesion** : A steering force towards the center of mass of nearby agents

**Alignment**: A steering force towards the direction of average velocity of nearby agents

**Sepeation**: A steering force pointing away from agents that are too close

The combination of boids rules produces a behavior which has desirable properties in navigation in multi-agent systems. These include obstacle avoidance, path planning and formation control

Some applications include:
1. Autonomous mobile robot control (selfdriving cars, warehouse drones, search and rescue drones, etc.)
2. Satellite formation control

Here is a boids flocking model implementation by Daniel Shiffman https://processing.org/examples/flocking.html

# How to Use
**Prerequisites:** Python3, pygame, numpy

Run the main file and use the following scheme to control the leader (red) to guide the followers (blue):

"W", "S" : Increase and decrease linear velocity respectively

"A", "D" : Increase angular velocity anticlockwise and clockwise respectively

<img width="936" alt="1 step robot control" src="https://user-images.githubusercontent.com/95622570/163323944-78a3668d-a629-4f12-b9f6-6da77b48959b.png">

**Green lines:** Exchange of state information (position velocity). 

**Red lines:** Direction of velocity. 

**Blue lines:** Avoidance method being active due to close proximity to other robot(s)

**Notes:** Performance depends on efficient control of leader as this does not use any future prediction optimization and therefore the limited motion of the robots causes them to collide if they are clustered together.

**Future work:** Include modern control methods like MPC (Model Predictive Control) where the future trajectories can be computed and adjusted accordingly for optimal control inputs
