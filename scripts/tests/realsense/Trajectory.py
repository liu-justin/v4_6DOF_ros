import numpy as np
import matplotlib.pyplot as plt
import fits as f

class Trajectory():
    def __init__(self, time, point):
        self.init_time = time
        self.times = [0]
        self.points = [point]
        self.beta1_x = 5
        self.beta0_x = 0
        self.beta1_x_low = 5
        self.beta0_x_low = 0
        self.beta1_x_high = 5
        self.beta0_x_high = 0
        self.beta2_y = -9.81
        self.beta1_y = 0
        self.beta0_y = self.points[0][1]
        self.beta1_z = 5
        self.beta0_z = 0

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
        self.beta1_x, self.beta0_x = f.linear_simple(self.points[0][0], new_point[0], self.times[0], time_delta)
        self.beta1_z, self.beta0_z = f.linear_simple(self.points[0][2], new_point[2], self.times[0], time_delta)

        if self.beta1_x <= 5 and self.beta1_z <= 5:
            self.times.append(time_delta)
            self.points.append(new_point)
            self.beta1_x_low, self.beta0_x_low, self.beta1_x_high, self.beta0_x_high = f.linear_errored(self.points[0][0], new_point[0], self.times[0], time_delta, 0.05, 0.05)
            self.beta1_z_low, self.beta0_z_low, self.beta1_z_high, self.beta0_z_high = f.linear_errored(self.points[0][2], new_point[2], self.times[0], time_delta, 0.05, 0.05)
            print("suceeded in first append")
            return True
        else:
            print("failed in first append")
            return False

    # sets the y betas and resets linear betas
    def appendSecond(self,new_time, new_point):
        time_delta = new_time - self.init_time
        predicted_x = self.beta0_x + self.beta1_x*time_delta
        predicted_z = self.beta0_z + self.beta1_z*time_delta
        print(f"appending into traj w/ length 1: predicted {predicted_x},{predicted_z}: actual {new_point[0]}, {new_point[2]}")
        if abs((new_point[0]-predicted_x)/predicted_x) < 0.1 and abs((new_point[2]-predicted_z)/predicted_z) < 0.1:
            self.times.append(time_delta)
            self.points.append(new_point)
            self.beta1_x, self.beta0_x = f.linear_least_squared(self.times, [p[0] for p in self.points]) # X
            self.beta2_y, self.beta1_y = f.poly_least_squared_beta0const(self.times, [p[1] for p in self.points], self.beta0_y) # Y
            self.beta1_z, self.beta0_z = f.linear_least_squared(self.times, [p[2] for p in self.points]) # Z
            print("succeeded in second append")
            return True
        else:
            print("failed in second append")
            return False

            
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

                self.beta1_x, self.beta0_x = f.linear_least_squared(self.times, [p[0] for p in self.points]) # X
                self.beta2_y, self.beta1_y = f.poly_least_squared_beta0const(self.times, [p[1] for p in self.points], self.beta0_y) # Y
                self.beta1_z, self.beta0_z = f.linear_least_squared(self.times, [p[2] for p in self.points]) # Z

                print(f"succeeded in into traj w/ length {len(self.times)}")
                self.plotY()
                return True

            else:
                print(f"failed in appending into traj w/ length {len(self.times)}")
                return False