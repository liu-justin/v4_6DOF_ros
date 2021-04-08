import numpy as np
import matplotlib.pyplot as plt
import fits as f
from collections import OrderedDict

class Trajectory():
    def __init__(self, time, point):
        self.init_time = time
        self.times = [0]
        self.points = [point]
        self.use_errored = [1,1,1]
        self.betas = OrderedDict()
        self.betas["x"] = [0,5,0]
        self.betas["y"] = [self.points[0][1],0,-9.81]
        self.betas["z"] = [0,5,0]

        self.betas_low = OrderedDict()
        self.betas_low["x"] = [0,5,0]
        self.betas_low["y"] = [self.points[0][1],0,-9.81]
        self.betas_low["z"] = [0,5,0]

        self.betas_high = OrderedDict()
        self.betas_high["x"] = [0,5,0]
        self.betas_high["y"] = [self.points[0][1],0,-9.81]
        self.betas_high["z"] = [0,5,0]


    def plotY(self):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        for p,t in zip(self.points, self.times):
            ax.scatter(t, p[1])
        x = np.arange(0,self.times[-1],0.01)
        y= self.beta0_y + self.beta1_y*x + self.beta2_y*(x**2)
        ax.plot(x,y)
        plt.show()

    # sets the linear betas
    def appendFirst(self, new_time, new_point):
        time_delta = new_time - self.init_time
        if (time_delta == 0): return False
        self.betas["x"] = f.linear_simple(new_point[0], time_delta,  self.points[0][0], self.times[0])
        self.betas["z"] = f.linear_simple(new_point[2], time_delta,  self.points[0][2], self.times[0])

        if self.betas["x"][1] <= 5 and self.betas["z"][1] <= 5:
            self.times.append(time_delta)
            self.points.append(new_point)
            # for in plane coord errors, couldn't really think of a good value: some of the error will come from the image being off, some will come from deproject_pixel_to_point, some from canny
            # ended up going for half the size of the ball as total error (20mm), so 10mm on each side
            # for time errors, couldn't find anything again, so just a random value ( i get a sample timestamp accuracy on p72, but that is for the imu)
            self.betas_low["x"], self.betas_high["x"] = f.linear_errored(new_point[0], time_delta,  self.points[0][0], self.times[0], 0.01, 0.001)
            self.betas_low["y"], self.betas_high["y"] = f.poly_errored(  new_point[1], time_delta,  self.points[0][1], self.times[0], 0.03, 0.001)
            self.betas_low["z"], self.betas_high["z"] = f.linear_errored(new_point[2], time_delta,  self.points[0][2], self.times[0], 0.01, 0.001)
            print("suceeded in first append")
            return True
        else:
            print("failed in first append")
            return False

    def predicted(self, betas, time):
        return betas[0] + betas[1]*time + betas[2]*(time**2)

    # sets the y betas and resets linear betas
    def appendSecond(self,new_time, new_point):
        time_delta = new_time - self.init_time
        predicted_x_low = self.predicted(self.betas_low["x"], time_delta)
        predicted_x_high = self.predicted(self.betas_high["x"], time_delta)
        predicted_y_low = self.predicted(self.betas_low["y"], time_delta)
        predicted_y_high = self.predicted(self.betas_high["y"], time_delta)
        predicted_z_low = self.predicted(self.betas_low["z"], time_delta)
        predicted_z_high = self.predicted(self.betas_high["z"], time_delta)
        print(f"appending into length 2: predicted x {predicted_x_low},{predicted_x_high}: actual {new_point[0]}")
        print(f"appending into length 2: predicted y {predicted_y_low},{predicted_y_high}: actual {new_point[1]}")
        print(f"appending into length 2: predicted z {predicted_z_low},{predicted_z_high}: actual {new_point[2]}")
        if predicted_x_low - 0.01 < new_point[0] < predicted_x_high + 0.01 and \
           predicted_z_low - 0.01 < new_point[2] < predicted_z_high + 0.01 and \
           predicted_y_low - 0.03 < new_point[1] < predicted_y_high + 0.03    :
            self.times.append(time_delta)
            self.points.append(new_point)

            # determine which fit to use (errored or least_squares): find the R2 for each axis, modify the use_errored list, then continue 
            # to find R2, I need to find the betas

            self.beta1_x, self.beta0_x = f.linear_least_squares(self.times, [p[0] for p in self.points]) # X
            self.beta2_y, self.beta1_y, self.beta0_y = f.poly_least_squares_beta0const(self.times, [p[1] for p in self.points], self.beta0_y) # Y
            self.beta1_z, self.beta0_z = f.linear_least_squares(self.times, [p[2] for p in self.points]) # Z
            print("succeeded in second append")
            return True
        else:
            print("failed in second append")
            return False

    def determineFit(self):
        self.betas["x"] = f.linear_least_squares(self.times, [p[0] for p in self.points]) # X
        self.betas["y"]  = f.poly_least_squares_beta0const(self.times, [p[1] for p in self.points], self.beta0_y) # Y
        self.betas["z"]  = f.linear_least_squares(self.times, [p[2] for p in self.points]) # Z

        RSS_x = sum([(self.points[i][0] - self.predicted(self.betas["x"], self.times[i]))**2] for i in range(len(self.points)))

            
    def append(self, new_time, new_point):
        if len(self.times) == 1:
            return self.appendFirst(new_time, new_point)
        elif len(self.times) == 2:
            return self.appendSecond(new_time, new_point)
        else:
            time_delta = new_time - self.init_time
            predicted_x = self.beta0_x + self.beta1_x*time_delta
            predicted_y = self.beta0_y + self.beta1_y*time_delta + self.beta2_y*(time_delta**2)
            predicted_z = self.beta0_z + self.beta1_z*time_delta

            print(f"appending into traj w/ length {len(self.times)}: predicted {predicted_x},{predicted_y},{predicted_z}: actual {new_point[0]}, {new_point[1]}, {new_point[2]}")

            if abs((new_point[0]-predicted_x)/predicted_x) < 0.1 and abs((new_point[1]-predicted_y)/predicted_y) < 0.5 and abs((new_point[2]-predicted_z)/predicted_z) < 0.1:
                self.times.append(time_delta)
                self.points.append(new_point)

                self.beta1_x, self.beta0_x = f.linear_least_squares(self.times, [p[0] for p in self.points]) # X
                self.beta2_y, self.beta1_y, self.beta0_y = f.poly_least_squares_beta0const(self.times, [p[1] for p in self.points], self.beta0_y) # Y
                self.beta1_z, self.beta0_z = f.linear_least_squares(self.times, [p[2] for p in self.points]) # Z

                print(f"succeeded in into traj w/ length {len(self.times)-1}, now length {len(self.times)}")
                # self.plotY()
                return True

            else:
                print(f"failed in appending into traj w/ length {len(self.times)}")
                return False

""" 
    # sets the y betas and resets linear betas
    def appendSecondSimple(self,new_time, new_point):
        time_delta = new_time - self.init_time
        predicted_x = self.beta0_x + self.beta1_x*time_delta
        predicted_y = self.beta0_y + self.beta1_y*time_delta + self.beta2_y*(time_delta**2)
        predicted_z = self.beta0_z + self.beta1_z*time_delta
        print(f"appending into traj w/ length 1: predicted {predicted_x},{predicted_z}: actual {new_point[0]}, {new_point[2]}")
        if abs((new_point[0]-predicted_x)/predicted_x) < 0.1 and \
           abs((new_point[1]-predicted_y)/predicted_y) < 0.2 and \
           abs((new_point[2]-predicted_z)/predicted_z) < 0.1:
            self.times.append(time_delta)
            self.points.append(new_point)
            self.beta1_x, self.beta0_x = f.linear_least_squares(self.times, [p[0] for p in self.points]) # X
            self.beta2_y, self.beta1_y = f.poly_least_squares_beta0const(self.times, [p[1] for p in self.points], self.beta0_y) # Y
            self.beta1_z, self.beta0_z = f.linear_least_squares(self.times, [p[2] for p in self.points]) # Z
            print("succeeded in second append")
            return True
        else:
            print("failed in second append")
            return False
            """