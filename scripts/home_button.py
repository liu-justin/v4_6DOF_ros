import modern_robotics as mr
import tkinter as tk
import numpy as np
import time

class HomeButton(tk.Frame):
    def __init__(self, window, motors):
        super().__init__(window)

        self.root = window
        self.motors = motors

        # parameters for joint trajectory
        self.T_final = 10
        self.N = 110          # # of times that it samples
        method = 5          # cubic or quintic sampling for mr

        theta_rest = np.array([0,0,0,0,0,0])
        theta_home = np.array([0,-1*np.pi/2, np.pi/2, 0,0,0])

        # create the trajectory, N x n matrix where each row is  n-vector of joint variables at an instant in time
        self.trajectory = mr.JointTrajectory(theta_rest, theta_home, self.T_final, self.N, method)

        self.rowconfigure(0, minsize=100, weight=1)
        self.columnconfigure([0], minsize=100, weight=1)
        self.button = tk.Button(master=self, text="home", command=self.home)
        self.button.grid(row=0,column=1)
        self.pack()

    def home(self):
        # get number of divisions to get from first angles to last angles
        individ_time = self.T_final/self.N

        # get number of motors
        num_motors = len(self.motors)

        # run thru angleList without first one
        for i in range(1, self.N):
            for j in range(num_motors):
                # get the angular velocity from the previous_angles
                angular_velocity = (self.trajectory[i][j] - self.trajectory[i-1][j])/individ_time
                
                # send the crap
                self.motors[j].velocity = angular_velocity

            # wait for the divisionTime (in ms)
            
            # self.root.after(int(individ_time*1000))
            time.sleep(individ_time)

        # after running thru this entire angle_list, send a zero velocity to stop the machine
        for motor in self.motors:
            motor.velocity = 0