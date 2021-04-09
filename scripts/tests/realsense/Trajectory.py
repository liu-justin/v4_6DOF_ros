import numpy as np
import matplotlib.pyplot as plt
import fits as f
from collections import OrderedDict

def dimToInt(dim):
    return ord(dim)-120 #ASCII for "x" is 120

class Trajectory():
    def __init__(self, time, point):
        self.init_time = time
        self.times = [0]
        self.points = [point]
        self.use_errored = [True,True,True]
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

    # # sets the linear betas
    def appendFirst(self, new_time, new_point):
        time_delta = new_time - self.init_time
        if (time_delta == 0): return False
        self.betas["x"] = f.linear_simple(new_point[0], time_delta,  self.points[0][0], self.times[0])
        self.betas["z"] = f.linear_simple(new_point[2], time_delta,  self.points[0][2], self.times[0])

        if self.betas["x"][1] <= 5 and self.betas["z"][1] <= 5:
            self.times.append(time_delta)
            self.points.append(new_point)
            for dim in self.betas.keys():
                self.findBetasErrored(dim)
            print("suceeded in first append")
            return True
        else:
            print("failed in first append")
            return False

    def findBetasErrored(self, dim):
        # for in plane coord errors, couldn't really think of a good value: some of the error will come from the image being off, some will come from deproject_pixel_to_point, some from canny
        # ended up going for half the size of the ball as total error (20mm), so 10mm on each side
        # for time errors, couldn't find anything again, so just a random value ( i get a sample timestamp accuracy on p72, but that is for the imu)
        if dim == "y":
            self.betas_low["y"], self.betas_high["y"] = f.poly_errored(  self.points[-1][dim_index], self.times[-1],  self.points[0][dimToInt(dim)], self.times[0], 0.03, 0.001)
        else:
            self.betas_low[dim], self.betas_high[dim] = f.linear_errored(self.points[-1][dim_index], self.times[-1],  self.points[0][dimToInt(dim)], self.times[0], 0.01, 0.001)

    def findBetasLeastSquared(self, dim):
        if dim == "y":
            self.betas["y"] = f.poly_least_squares_beta0const(self.times, [p[1] for p in self.points], self.betas["y"][0])
        else:
            self.betas[dim] = f.linear_least_squares(self.times, [p[dimToInt(dim)] for p in self.points])

    def predicted(self, betas, time):
        return betas[0] + betas[1]*time + betas[2]*(time**2)

    def determineFit(self, dim):
        self.betas[dim] = self.findBetasLeastSquared(dim)

        # https://stats.stackexchange.com/questions/219810/r-squared-and-higher-order-polynomial-regression
        RSS = sum([(p[dimToInt(dim)] - self.predicted(self.betas[dim], t))**2] for p,t in zip(self.points, self.times))
        bar = sum([p[dimToInt(dim)] for p in self.points])/len(self.points)
        TSS = sum([(p[dimToInt(dim)] - bar)**2 for p in self.points])
        R2 = 1 - RSS/TSS
        if R2 < 0.75:
            self.use_errored[dimToInt(dim)] = False
        else:
            self.findBetasErrored(dim)
            
    def append(self, new_time, new_point):
        if len(self.times) == 1:
            return self.appendFirst(new_time, new_point)
        else:
            success = [False, False, False]
            for dim in self.betas.keys():
                if self.use_errored[dimToInt(dim)]:
                    success[i] = self.useErrored(new_time, new_point, dim)
                else:
                    success[i] = self.useLeastSquares(new_time, new_point, dim)
            
            if all(success):
                self.times.append(time_delta)
                self.points.append(new_point)
                for dim in self.betas.keys():
                    if self.use_errored[dimToInt(dim)]:
                        self.determineFit(dim)
                        

    def useErrored(self, new_time, new_point, dim):
        time_delta = new_time - self.init_time
        predicted_low = self.predicted(self.betas_low[dim], time_delta)
        predicted_high = self.predicted(self.betas_high[dim], time_delta)
        if dim =="y": error = 0.03
        else: error = 0.01

        if (predicted_low - error) < new_point[dimToInt(dim)] < (predicted_high + error): return True
        else: return False
            

    def useLeastSquares(self, new_time, new_point, dim):
        time_delta = new_time - self.init_time
        predicted = self.predicted(self.betas[dim], time_delta)
        if dim =="y": error = 0.3
        else: error = 0.15

        if abs(new_point[dimToInt(dim)] - predicted)/predicted < error: return True
        else: return False

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

"""
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
            self.determineFit()
            print("succeeded in second append")
            return True
        else:
            print("failed in second append")
            return False"""