import numpy as np
import matplotlib.pyplot as plt

def linear_init(x1, x2, t1, t2):
    beta1 = (x2 - x1)/(t2 - t1) # BL2_TR1_m
    beta0 = x1 - beta1*t1
    return beta1,beta0

def poly_init_beta2const(x1, x2, t1, t2):
    beta2 = -9.81
    beta1 = (x2-x1)/(t2-t1) - beta2*(t2+t1)
    beta0 = x1 - beta2*(t1**2) - beta1*t1
    return beta1,beta0

def linear_least_squared(x, y):
    n = len(x)
    x_bar = sum(x)/n
    y_bar = sum(y)/n
    Sxy = sum([(yi-y_bar)*xi for xi,yi in zip(x,y)])
    Sxx = sum([(xi-x_bar)*xi for xi in x])
    beta1 = Sxy/Sxx
    beta0 = y_bar - beta1*x_bar

    return beta1, beta0

def poly_least_squared_beta2const(x, y): # derivation in google drive
    n = len(x)
    g = -9.81
    x_bar = sum(x)/n
    x2_bar = sum([xi**2 for xi in x])/n
    y_bar = sum(y)/n
    Sxy = sum([(yi-y_bar)*xi for xi,yi in zip(x,y)])
    Sxx = sum([(xi-x_bar)*xi for xi in x])
    g_sum = g * sum([xi*(xi**2-x2_bar) for xi in x])
    beta1 = (Sxy - g_sum)/Sxx
    beta0 = y_bar - beta1*x_bar - g*x2_bar

    return beta1, beta0

def poly_least_squared_beta0const(x, y, beta0):
    sum_xi = sum(x)
    sum_xi2 = sum([xi**2 for xi in x])
    sum_xi3 = sum([xi**3 for xi in x])
    sum_xi4 = sum([xi**4 for xi in x])
    sum_xiyi = sum([xi*yi for xi,yi in zip(x,y)])

    numerator_1 = sum([(xi**2)*yi for xi,yi in zip(x,y)])
    numerator_2 = beta0 * sum_xi2
    numerator_3 = sum_xiyi * sum_xi3 / sum_xi2
    numerator_4 = beta0 * (sum_xi/sum_xi2) * sum_xi3

    denom = -1*(sum_xi3*sum_xi3/sum_xi2 - sum_xi4)

    beta2 = (numerator_1 - numerator_2 - numerator_3 + numerator_4)/denom
    beta1 = (sum_xiyi - beta0*sum_xi - beta2*sum_xi3)/sum_xi2

    return beta2, beta1

"""
    t = [0, 0.0333399772644043, 0.06669783592224121]
    y = [0.03290366, 0.06382939, 0.08356368]

    beta1, beta0 = poly_least_squared_beta2const(t,y)
    beta1_a, beta0_a = poly_init_beta2const(y[0], y[1], t[0], t[1])
    beta2_b, beta1_b = poly_least_squared_beta0const(t,y,y[0])
    print(f"{beta1}, {beta0}")
    print(f"{beta1_a}, {beta0_a}")
    print(f"{beta2_b}, {beta1_b}")

    fig = plt.figure()
    ax = fig.add_subplot(111)
    for yi,ti in zip(y, t):
        ax.scatter(ti, yi)
    x = np.arange(0,t[-1],0.01)
    y= y[0] + beta1_b*x + beta2_b*(x**2)
    ax.plot(x,y)
    plt.show()
"""


class Trajectory():
    def __init__(self, time, point):
        self.init_time = time
        self.times = [0]
        self.points = [point]
        self.beta1_x = 5
        self.beta0_x = 0
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

    def appendFirst(self, new_time, new_point):
        time_delta = new_time - self.init_time
        self.beta1_x, self.beta0_x = linear_init(self.points[0][0], new_point[0], self.times[0], time_delta)
        self.beta1_z, self.beta0_z = linear_init(self.points[0][2], new_point[2], self.times[0], time_delta)

        if self.beta1_x <= 5 and self.beta1_z <= 5:
            self.times.append(time_delta)
            self.points.append(new_point)
            print("suceeded in first append")
            return True
        else:
            print("failed in first append")
            return False

    def appendSecond(self,new_time, new_point):
        time_delta = new_time - self.init_time
        predicted_x = self.beta0_x + self.beta1_x*time_delta
        predicted_z = self.beta0_z + self.beta1_z*time_delta
        if abs((new_point[0]-predicted_x)/predicted_x) < 0.1 and abs((new_point[2]-predicted_z)/predicted_z) < 0.1:
            self.times.append(time_delta)
            self.points.append(new_point)
            self.beta1_x, self.beta0_x = linear_least_squared(self.times, [p[0] for p in self.points]) # X
            self.beta2_y, self.beta1_y = poly_least_squared_beta0const(self.times, [p[1] for p in self.points], self.beta0_y) # Y
            self.beta1_z, self.beta0_z = linear_least_squared(self.times, [p[2] for p in self.points]) # Z
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
            predicted_y = self.beta0_y + self.beta1_y*time_delta + -9.81*(time_delta**2)
            predicted_z = self.beta0_z + self.beta1_z*time_delta

            print(f"appending into traj w/ length {len(self.times)}: predicted {predicted_x},{predicted_y},{predicted_z}")
            print(f"appending into traj w/ length {len(self.times)}: actual {new_point[0]}, {new_point[1]}, {new_point[2]}")

            if abs((new_point[0]-predicted_x)/predicted_x) < 0.1 and abs((new_point[1]-predicted_y)/predicted_y) < 0.5 and abs((new_point[2]-predicted_z)/predicted_z) < 0.1:
                self.times.append(time_delta)
                self.points.append(new_point)

                self.beta1_x, self.beta0_x = linear_least_squared(self.times, [p[0] for p in self.points]) # X
                self.beta2_y, self.beta1_y = poly_least_squared_beta0const(self.times, [p[1] for p in self.points], self.beta0_y) # Y
                self.beta1_z, self.beta0_z = linear_least_squared(self.times, [p[2] for p in self.points]) # Z

                print(f"succeeded in into traj w/ length {len(self.times)}")
                self.plotY()
                return True

            # need to figure out least squared fitting https://www.mathsisfun.com/data/least-squares-regression.html
            # https://mathworld.wolfram.com/LeastSquaresFitting.html
            else:
                print(f"failed in appending into traj w/ length {len(self.times)}")
                return False