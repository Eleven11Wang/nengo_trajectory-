import math
import numpy as np
import nengo
class nengoSource(object):
    def __init__(self,x,y,heading,max_time,
                acceleration=0.2,
                drag=0.4):
        self.heading=heading
        self.velocity=[0,0]
        self.acceleration=acceleration
        self.drag=drag
        self.x=x
        self.y=y
        self.max_time=max_time
        self.sensor_angle=45 * np.pi / 180
        self.sensor_dist=0.3
        self.dt=0.001
        self.source=[0,1]
        self.trail=[]
        self.trail_length=0
    def sensor_pos(self):
        left_x = self.x + self.sensor_dist * np.cos(self.heading - self.sensor_angle)
        left_y = self.y + self.sensor_dist * np.sin(self.heading - self.sensor_angle)
        right_x = self.x + self.sensor_dist * np.cos(self.heading + self.sensor_angle)
        right_y = self.y + self.sensor_dist * np.sin(self.heading + self.sensor_angle)
        sensor_pos_array=(left_x, left_y, right_x, right_y)
        return sensor_pos_array
    def distance_from_source(self,sensor_pos_array): 
        left_x, left_y, right_x, right_y= sensor_pos_array
        source=self.source
        left_distance=((source[0]-left_x)**2 + (source[1]-left_y)**2)**0.5
        right_distance=((source[0]-right_x)**2 + (source[1]-right_y)**2)**0.5
        return self.sigmoid(left_distance),self.sigmoid(right_distance)
    def distance_to_binaray(self,left_distance,right_distance): 
        if left_distance < right_distance:
            left_signal,right_signal = 1,0 
        else: 
            left_signal,right_signal=0,1
        return left_signal,right_signal
        
    def sigmoid(self,x):
        return 1/(1+math.exp(-x))
        
    def make_source_move(self):
        init_source=self.source
        dt=self.dt
        def update(t):
            if t >= 0.8 * self.max_time:
                pass
            else:
                self.source[0]=np.sin(1/2 * np.pi * -t/self.max_time)
                self.source[1]=np.cos(1/2 * np.pi* -t/self.max_time)
            return (self.source)
        return nengo.Node(update)
        
    
    def make_sensor(self):
        self.last_update = -9999
        dt = self.dt
        def update(t):
            if t >= self.last_update + dt:
                self.last_update = t
                
            sensor_pos_array = self.sensor_pos()
            left_distance,right_distance=self.distance_from_source(sensor_pos_array)
            left_signal,right_signal=self.distance_to_binaray(left_distance,right_distance)
            if (t * 1000) % 10 != 0:
                left_signal,right_signal = 0,0

            return (left_signal,right_signal)
            
        return nengo.Node(update)
    
    
    def rotate(self,theta, r):
        """Return new heading after a rotation around Z axis."""
        #return (theta + r + np.pi) % (2.0 * np.pi) - np.pi
        return (theta + r + np.pi)  - np.pi


    def thrust(self,theta, acceleration):
        """Thrust vector from current heading and acceleration

        theta: clockwise radians around z-axis, where 0 is forward
        acceleration: float where max speed is ....?!?
        """
        return np.array([np.sin(theta), np.cos(theta)]) * acceleration


    def get_next_state(self,heading, velocity, rotation, acceleration, drag=0.5):
        """Get new heading and velocity, based on relative rotation and
        acceleration and linear drag."""
        theta = self.rotate(heading, rotation)
        v = velocity + self.thrust(theta, acceleration)
        v -= drag * v
        return theta, v
    
    def make_movement(self, dt=0.001):
        def update(t, x):
            
            
            self.heading, self.velocity = self.get_next_state( heading=self.heading, velocity=self.velocity,
                rotation=x[0] * dt,
                
                acceleration=self.acceleration,
                drag=self.drag)
            self.x += self.velocity[1] * dt
            self.y += self.velocity[0] * dt

            if self.trail_length > 0:
                if self.last_trail_time is None or t > self.last_trail_time + self.trail_dt:
                    self.trail.append((self.x, self.y))
                    if len(self.trail) > self.trail_length:
                        self.trail = self.trail[1:]
                    self.last_trail_time = t
            return x

        return nengo.Node(update, size_in=1)
    def avoid_block(self):
        def update(t,x): 
            if self.x < 0.1 and self.y < 0.1: 
                return -0.1 
            else: 
                return x
        return nengo.Node(update, size_in=1)
    def make_position(self):
        def update(t):
            return self.x, self.y

        return nengo.Node(update)
    
        
    def make_heading_and_velocity(self):
        def update(t):
            return self.heading, self.velocity[0], self.velocity[1]

        return nengo.Node(update, size_in=0, size_out=3)
