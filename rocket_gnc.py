import numpy as np
import matplotlib.pyplot as plt
import turtle
import time

# Constants
SETPOINT_Y = 300
SETPOINT_X = 100
TIME_STEP = 0.001
SIM_TIME = 1000
INITIAL_Y = -100
INITIAL_X = 0
MASS = 1
MAX_THRUST = 15
MAX_THRUST_X = 2
g = -9.81

# PID constants for Y-direction
KP_Y = 0.24
KI_Y = 20.57
KD_Y = 0.00157

# PID constants for X-direction
KP_X = 0.24
KI_X = 20.57
KD_X = 0.00157

class Simulation:
    def __init__(self):
        self.rocket = Rocket()
        self.pid_y = PID(KP_Y, KI_Y, KD_Y, SETPOINT_Y)
        self.pid_x = PIDX(KP_X, KI_X, KD_X, SETPOINT_X)
        self.screen = turtle.Screen()
        self.screen.setup(1920, 1080)
        
        self.marker = turtle.Turtle()
        self.marker.penup()
        self.marker.goto(SETPOINT_X, SETPOINT_Y)
        self.marker.color('blue')
        self.sim = True
        self.timer = 0
        self.poses_x = []
        self.poses_y = []
        self.times = []

    def cycle(self):
        while self.sim:
            thrust_y = self.pid_y.compute(self.rocket.get_y())
            thrust_x = self.pid_x.compute(self.rocket.get_x())
            self.rocket.update(thrust_y, thrust_x)
            time.sleep(TIME_STEP)
            self.timer += 1

            if self.timer > SIM_TIME or not (-800 < self.rocket.get_y() < 800): # or not (-800 < self.rocket.get_x() < 800):
                print("SIM ENDED")
                self.sim = False

            self.poses_y.append(self.rocket.get_y())
            self.poses_x.append(self.rocket.get_x())
            self.times.append(self.timer)

        self.graph(self.times, self.poses_y, 'Y Position')
        self.graph(self.times, self.poses_x, 'X Position')

    @staticmethod
    def graph(x, y, label):
        plt.plot(x, y)
        plt.title(label)
        plt.xlabel('Time (ms)')
        plt.ylabel(label)
        plt.show()

class Rocket:
    def __init__(self):
        self.turtle = turtle.Turtle()
        self.turtle.shape('square')
        self.turtle.color('black')
        self.turtle.goto(INITIAL_X, INITIAL_Y)
        self.turtle.speed(0)

        self.dy = 0
        self.dx = 0

    def update(self, thrust_y, thrust_x):
        ddy = g + thrust_y / MASS
        ddx = thrust_x / MASS
        self.dy += ddy
        self.dx += ddx
        self.turtle.sety(self.turtle.ycor() + self.dy)
        self.turtle.setx(self.turtle.xcor() + self.dx)

    def get_y(self):
        return self.turtle.ycor()

    def get_x(self):
        return self.turtle.xcor()

class PID:
    def __init__(self, KP, KI, KD, target):
        self.kp = KP
        self.ki = KI
        self.kd = KD
        self.setpoint = target

        self.integral_error = 0
        self.error_last = 0

    def compute(self, pos):
        error = self.setpoint - pos
        self.integral_error += error * TIME_STEP
        derivative_error = (error - self.error_last) / TIME_STEP
        self.error_last = error
        output = self.kp * error + self.ki * self.integral_error + self.kd * derivative_error
        
        return max(0, min(MAX_THRUST, output))
    
            
class PIDX:
    def __init__(self, KP, KI, KD, target):
        self.kp = KP
        self.ki = KI
        self.kd = KD
        self.setpoint = target

        self.integral_error = 0
        self.error_last = 0

    def compute(self, pos):
        error = self.setpoint - pos
        self.integral_error += error * TIME_STEP
        derivative_error = (error - self.error_last) / TIME_STEP
        self.error_last = error
        output = self.kp * error + self.ki * self.integral_error + self.kd * derivative_error
        return min(MAX_THRUST_X, output)


def main():
    sim = Simulation()
    sim.cycle()

main()
