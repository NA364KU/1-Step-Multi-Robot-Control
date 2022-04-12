import pygame
import math
import numpy as np
import os
PI = math.pi

class Robot:
    def __init__(self,inital_pos,image,state = None,id = None):
        self.leader = state
        self.a = 20 #diameter of object
        self.ID = id
        self.x,self.y = inital_pos
        self.theta = 0
        self.u = 30 #pix/sec
        self.omega = 0 #rad/sec
        self.img = pygame.image.load(image)
        self.rotated = self.img
        self.rect = self.rotated.get_rect(center=(self.x,self.y))
        self.m2p = 3.77952 #meters to pixel conversion

# manual control of leader use W-S to change linear speed and A-D to change angular speed
    def move(self,event = None):
        self.x += (self.u*math.cos(self.theta) - self.a*math.sin(self.theta)*self.omega)*dt
        self.y += (self.u*math.sin(self.theta) + self.a*math.cos(self.theta)*self.omega)*dt
        self.theta += self.omega*dt
        self.rotated = pygame.transform.rotozoom(self.img,math.degrees(-self.theta),1)
        self.rect = self.rotated.get_rect(center=(self.x,self.y))
        if self.leader:
            if event is not None:
                if True:
                    if event[pygame.K_w]:
                        self.u += 0.01*self.m2p
                    elif event[pygame.K_s]:
                        self.u -= 0.01*self.m2p
                    elif event[pygame.K_d]:
                        self.omega += 0.001 * self.m2p
                    elif event[pygame.K_a]:
                        self.omega -= 0.001 * self.m2p

    def dist(self,p1,p2):
        (x1,y1) = p1
        (x2,y2) = p2
        x1 = float(x1)
        y1 = float(y1)
        x2 = float(x2)
        y2 = float(y2)
        px = (x1 - x2) ** 2
        py = (y1 - y2) ** 2
        distance = (px + py) ** 0.5
        return distance

    def draw(self,map):
        map.blit(self.rotated,self.rect)

def two_points_to_line(point1,point2):
    x1,y1 = point1
    x2,y2 = point2
    if x1==x2:
        return 0
    slope = (y2-y1)/(x2-x1)
    c = y2 - (slope*x2)
    line = slope,c
    return line

def intersection_point(line1,line2):
    a,c = line1
    b,d = line2
    if a==b:
        return 0
    px = (d-c)/(a-b)
    py = a*(px) + c
    point = px,py
    return point


def automove(robots,target_num,dt,Environment):
    pos1 = (robots[target_num].x, robots[target_num].y)
    pos2 = (robots[0].x, robots[0].y)
    p1 = 0
    p2 = 0
    n = len(robots)

    #Alignment

    #Finding average velocity for alignment
    for i in range(n-1):
        current_pos = (robots[i+1].x,robots[i+1].y)
        current_dist = robots[0].dist(pos1,current_pos)
        if current_dist < 300: #300 is minimum distance for alliance communication
            Environment.trace_neighbour(current_pos, pos1,(0,255,0))
            p1 += robots[i].u * math.cos(robots[i].theta) * (2-0.006*current_dist)
            p2 += robots[i].u * math.sin(robots[i].theta) * (2-0.006*current_dist)
    p1 = (p1/n)**2
    p2 = (p2/n)**2
    pf = (p1 + p2)**0.5

    # adjusting based on distance to speedup if far and slow if closer
    current_dist = robots[0].dist(pos1,pos2)
    pf = pf*current_dist/200

    #Linear velocity saturation function
    if pf > 50:
        robots[target_num].u = 50
    else:
        robots[target_num].u = pf

    #finding average direction for alignment
    t1 = 0
    t2 = 0
    n = len(robots)
    for i in range(n-1):
        current_pos = (robots[i+1].x, robots[i+1].y)
        current_dist = robots[0].dist(pos1, current_pos)
        if current_dist < 300:
            t1 += robots[i].u * math.cos(robots[i].theta)
            t2 += robots[i].u * math.sin(robots[i].theta)
    tf = t2/t1

    if robots[target_num].theta-math.atan(tf)>0:
        robots[target_num].theta -= 0.001
    if robots[target_num].theta-math.atan(tf)<0:
        robots[target_num].theta += 0.001

# Cohesion - steering towards leader

    temp = (robots[target_num].y - robots[0].y)/(robots[target_num].x - robots[0].x)
    steer_check = np.arctan(temp)
    current_theta = robots[target_num].theta

    #fixes bug of not steering correctly in 3 quadrants need to add 1 more fix
    if robots[target_num].x - robots[0].x >0 and robots[target_num].y - robots[0].y<0:
        steer_check = PI + steer_check
    if robots[target_num].x - robots[0].x >0 and robots[target_num].y - robots[0].y>0:
        steer_check = PI + steer_check

    cohesion = True

    if cohesion:
        if current_theta>2*PI:
            steer1 = steer_check / (2*PI)
            steer1 = steer1 % 1
            steer_check = 2*PI * steer1
        if current_theta<-2*PI:
            steer1 = steer_check / (2*PI)
            steer1 = steer1 % 1
            steer_check = 2*PI * steer1

        if steer_check > current_theta:
            robots[target_num].theta += 0.004
        if steer_check < current_theta:
            robots[target_num].theta -= 0.004

# Separation
    avoidance = True

    # Velocity vector based avoidance

    if avoidance:
        # forloop through robots[], check intersection point of velocity vectors, use the current position and theta to
        # get y = mx + c form and get point, check smallest distance from point to any of the robots, if it is
        # smaller than the safety, apply avoidance. This will be changing the steering angle.
        n = len(robots)
        for i in range(n - 1):
            current_pos = (robots[i + 1].x, robots[i + 1].y)
            current_dist = robots[0].dist(pos1, current_pos)
            do_vector = True
            if current_dist < 100 and do_vector:
                Environment.trace_neighbour(current_pos, pos1, (0, 0,255))
                current_pos_2 = (robots[i + 1].x + 5 * math.cos(robots[i+1].theta), robots[i + 1].y + 5 * math.sin(robots[i+1].theta))
                pos1_2 = (robots[target_num].x + 5 * math.cos(robots[target_num].theta), robots[target_num].y + 5 * math.sin(robots[target_num].theta))
                line1 = two_points_to_line(current_pos,current_pos_2)
                line2 = two_points_to_line(pos1,pos1_2)
                i_p = intersection_point(line1,line2)
                if i_p==0:
                    continue
                i_dist_1 = robots[0].dist(i_p,pos1)
                i_dist_2 = robots[0].dist(i_p,current_pos)
                if i_dist_1 < 100 or i_dist_2 < 100:
                    slope,c = line2
                    RHS = slope*robots[i+1].x + c
                    LHS = robots[i+1].y
                    if LHS > RHS:
                        robots[i+1].theta += 0.004
                        robots[target_num].theta -= 0.004
                    if RHS > LHS:
                        robots[i + 1].theta -= 0.004
                        robots[target_num].theta += 0.004

            #Charge particle type avoidance

            forces = True
            if current_dist < 100 and forces:
                Environment.trace_neighbour(current_pos, pos1, (0, 0, 255))
                #draw line between current and other agent
                #get angle of line using 2 points, find angle wrt to current agent. That is the target
                #line3 = two_points_to_line(current_pos,pos1)
                if robots[target_num].x - robots[i + 1].x==0:
                    continue
                if PI - math.tanh((robots[target_num].y - robots[i + 1].y) / (robots[target_num].x - robots[i + 1].x)) < robots[target_num].theta:
                    robots[target_num].theta += 0.002*(1-0.01*current_dist)
                if PI - math.tanh((robots[target_num].y - robots[i + 1].y) / (robots[target_num].x - robots[i + 1].x)) > robots[target_num].theta:
                    robots[target_num].theta -= 0.002*(1-0.01*current_dist)
                #figure out which direction to rotate

    # State update using unicycle model
    robots[target_num].x += (robots[target_num].u * math.cos(robots[target_num].theta) - robots[target_num].a * math.sin(robots[target_num].theta) * robots[target_num].omega) * dt
    robots[target_num].y += (robots[target_num].u * math.sin(robots[target_num].theta) + robots[target_num].a * math.cos(robots[target_num].theta) * robots[target_num].omega) * dt

    #Display update
    robots[target_num].rotated = pygame.transform.rotozoom(robots[target_num].img, math.degrees(-robots[target_num].theta), 1)
    robots[target_num].rect = robots[target_num].rotated.get_rect(center = (robots[target_num].x, robots[target_num].y))
    robots[target_num].draw(environment.map)
    Environment.trace((robots[target_num].x, robots[target_num].y),robots[target_num].theta)

class Environment:
    def __init__(self,dimensions):
        self.white = (255,255,255)
        self.height,self.width = dimensions
        pygame.display.set_caption("simulation test")
        self.map = pygame.display.set_mode((self.width,self.height))

    def trace(self, pos, rotation):
        n = 100
        centerx, centery = pos
        x_axis = (centerx + n * math.cos(rotation), centery + n * math.sin(rotation))
        pygame.draw.line(self.map, (255, 0, 0), (centerx, centery), x_axis, 1)

    def trace_neighbour(self,current_pos,neighbour_pos,colour):
        pygame.draw.line(self.map, colour, current_pos,neighbour_pos, 1)

def robot_simulate(Robot,event=None):
    Robot.move(event=event)
    Robot.draw(environment.map)

# Main Loop

pygame.init()

running = True
iterations = 0
dt = 0
lasttime = pygame.time.get_ticks()

start1 = (500,200)
start2 = (200,100)
start3 = (200,200)
start5 = (200,300)
start4 = (700,100)
start6 = (700,200)
start7 = (700,300)
start8 = (700,400)

start = [start1,start2,start3,start4,start5,start6,start7,start8]
dims = (600,1250)
environment = Environment(dims)
leader_image = os.getcwd() + r"\robot_leader.png"
follower_image = os.getcwd() + r"\robot_image.png"
robots = []
n = 8
for i in range(n):
    if i==0:
        a = Robot(start[i], leader_image, True, i)
    else:
        a = Robot(start[i],follower_image,True,i)
    robots.append(a)

# circular motion
robots[0].omega = 0.5
robots[0].u = 80

# Animation loop
while running:
    pygame.event.pump()
    keypressed = pygame.key.get_pressed()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        robot_simulate(robots[0], keypressed)
        for i in range(n-1):
            automove(robots,i+1,dt,environment)

    dt = (pygame.time.get_ticks() - lasttime) / 1000
    lasttime = pygame.time.get_ticks()

    pygame.display.update()
    environment.map.fill(environment.white)
    robot_simulate(robots[0],keypressed)

    for i in range(n-1):
        automove(robots, i + 1,dt,environment)
    iterations+=1





