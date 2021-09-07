import numpy as np
import matplotlib.pyplot as plt
from . import fits as f
from collections import OrderedDict
from . import modern_robotics as mr

def dimToInt(dim):
    return ord(dim)-120 #ASCII for "x" is 120

def intToDim(num): # 0 -> "x"
    return chr(num+120)

ERRORS = [0.04, 0.04, 0.04]

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

        self.avg_residuals = OrderedDict()
        self.avg_residuals["x"] = 0
        self.avg_residuals["y"] = 0
        self.avg_residuals["z"] = 0

        self.betas_low = OrderedDict()
        self.betas_low["x"] = [0,5,0]
        self.betas_low["y"] = [self.points[0][1],0,-9.81]
        self.betas_low["z"] = [0,5,0]

        self.betas_high = OrderedDict()
        self.betas_high["x"] = [0,5,0]
        self.betas_high["y"] = [self.points[0][1],0,-9.81]
        self.betas_high["z"] = [0,5,0]

        self.__checked = True
        self.ttl = 5

    @property
    def checked(self):
        return self.__checked
    
    @checked.setter
    def checked(self, incoming_bool):
        self.__checked = incoming_bool

    def detectHit(self):
        # detecting if ball hits a board at x=0, y=(-0.16, 0.05), z=(-0.284,-0.06)

        t = -1*self.betas["x"][0]/self.betas["x"][1]
        hit_y = self.betas["y"][0] + self.betas["y"][1]*t + self.betas["y"][2]*(t**2)
        hit_z = self.betas["z"][0] + self.betas["z"][1]*t + self.betas["z"][2]*(t**2)

        print(f"hit: {hit_y}, {hit_z}")

        if (-0.2 < hit_y < 0.09 and -0.324 < hit_z < -0.02):
            print("HIT")
            return True
        else:
            print("MISS")
            return False

    # plot least_squares line, low_error, high_error, and all the points
    def plotErrors(self, dim, new_time, new_point):
        time_delta = new_time - self.init_time
        int_dim = dimToInt(dim)

        fig = plt.figure()
        ax = fig.add_subplot(111)
        for p,t in zip(self.points, self.times):
            ax.scatter(t, p[int_dim])
        ax.scatter(new_time, new_point[int_dim])
        x = np.arange(0,time_delta,0.01)
        least_squares = self.betas[dim][0] + self.betas[dim][1]*x + self.betas[dim][2]*(x**2)
        low_errored = self.betas_low[dim][0] + self.betas_low[dim][1]*x + self.betas_low[dim][2]*(x**2)
        high_errored = self.betas_high[dim][0] + self.betas_high[dim][1]*x + self.betas_high[dim][2]*(x**2)
        ax.plot(x,least_squares, "r-" if self.use_errored[int_dim] else "g-")
        ax.plot(x,low_errored, "g-" if self.use_errored[int_dim] else "r-")
        ax.plot(x,high_errored, "g-" if self.use_errored[int_dim] else "r-")
        plt.show()

    def getPredictedPosition(self, betas, time):
        return betas[0] + betas[1]*time + betas[2]*(time**2)

    def append(self, new_time, new_point):
        if len(self.times) == 1:
            return self.appendFirst(new_time, new_point)
        else:
            # setting up success bools to see if the new_point fits this traj in xyz
            success = [False, False, False]

            # for all dimensions, use the appropriate fit to check the new_point
            for dim in self.betas.keys():
                if self.use_errored[dimToInt(dim)]:
                    success[dimToInt(dim)] = self.checkWithErrored(new_time, new_point, dim)
                else:
                    success[dimToInt(dim)] = self.checkWithLeastSquares(new_time, new_point, dim)
            
            # if new_point fits with all 3 dimensions, append it
            if all(success):
                print(f"succeeded in appending to list length {len(self.times)}")
                time_delta = new_time - self.init_time
                self.times.append(time_delta)
                self.points.append(new_point)
                self.checked = False

                # check thru all dimensions and choose the fit: leastsquares, R2
                for dim in self.betas.keys():
                    self.determineFit(dim)        
                return True

            # name and blame, and show the graph
            else:
                # for i in range(len(success)):
                #     if not success[i]:
                #         self.plotErrors(intToDim(i), new_time, new_point)
                return False

    # sets the linear betas
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
            return True
        else:
            return False

    def checkWithErrored(self, new_time, new_point, dim):
        time_delta = new_time - self.init_time
        predicted_low = self.getPredictedPosition(self.betas_low[dim], time_delta)
        predicted_high = self.getPredictedPosition(self.betas_high[dim], time_delta)
        error = 0.05 if dim=="y" else 0.02
        # if the point falls btwn the upper and lower bounds
        if (predicted_low - ERRORS[dimToInt(dim)]) < new_point[dimToInt(dim)] < (predicted_high + ERRORS[dimToInt(dim)]): 
            return True
        else:
            # print(f"{dim} failed: {predicted_low - error},{new_point[dimToInt(dim)]},{predicted_high + error}")
            return False

    def checkWithLeastSquares(self, new_time, new_point, dim):
        time_delta = new_time - self.init_time
        predicted = self.getPredictedPosition(self.betas[dim], time_delta)
        error = self.avg_residuals[dim]
        predicted_low = predicted - error
        predicted_high = predicted + error
        # if the new_point falls within a percentage of the predicted
        if (predicted_low - ERRORS[dimToInt(dim)]) < new_point[dimToInt(dim)] < (predicted_high + ERRORS[dimToInt(dim)]): 
            return True
        else:
            # print(f"{dim} failed: predicted{predicted} new_point{new_point[dimToInt(dim)]}")
            return False

    def findBetasErrored(self, dim):
        """finds the betas of this dimension using the square crisscross method
        :param dim: the current dim
        """
        if dim == "y":
            self.betas_low["y"], self.betas_high["y"] = f.poly_errored(self.points[-1][dimToInt(dim)], \
                                                                       self.times[-1],  \
                                                                       self.points[0][dimToInt(dim)], \
                                                                       self.times[0], 0.03, 0.001)
        else: 
            self.betas_low[dim], self.betas_high[dim] = f.linear_errored(self.points[-1][dimToInt(dim)], \
                                                                         self.times[-1], \
                                                                         self.points[0][dimToInt(dim)], \
                                                                         self.times[0], 0.01, 0.001)

    def findBetasLeastSquared(self, dim):
        """finds the betas of this dimension using the least squares regression method
        :param dim: the current dim
        """
        if dim == "y":
            self.betas["y"], self.avg_residuals["y"] = f.poly_least_squares_beta0const(self.times, \
                                                                                       [p[1] for p in self.points], \
                                                                                       self.betas["y"][0])
        else:
            self.betas[dim], self.avg_residuals[dim] = f.linear_least_squares(self.times, \
                                                                              [p[dimToInt(dim)] for p in self.points])

    def determineFit(self, dim):
        self.findBetasLeastSquared(dim)

        # https://stats.stackexchange.com/questions/219810/r-squared-and-higher-order-polynomial-regression
        RSS = sum([(p[dimToInt(dim)] - self.getPredictedPosition(self.betas[dim], t))**2 for p,t in zip(self.points, self.times)])
        bar = sum([p[dimToInt(dim)] for p in self.points])/len(self.points)
        TSS = sum([(p[dimToInt(dim)] - bar)**2 for p in self.points])
        R2 = 1 - RSS/TSS
        # if the coefficient of determination is high enough (least squares is good enough)
        if R2 > 0.95: 
            self.use_errored[dimToInt(dim)] = False
            # print("switching to least squares")
        else:
            self.use_errored[dimToInt(dim)] = True
            self.findBetasErrored(dim) 
            # print(f"staying with errored: R2 was {R2}")

    def getCoordFromTime(self, dim, time):
        return self.betas[dim][0] + self.betas[dim][1]*time + self.betas[dim][2]*(time**2)

    # current position of the hand
    def findClosestPointToM(self, M_current):
        """finds a point on the trajectory the minimum distance from the current location
        :param M_current: the current position of the robot head
        :return: success, the point where intersection occurs, the time it occurs
        Example Input:
            R = np.array([[1, 0,  0],
                        [0, 0, -1],
                        [0, 1,  0]])
            p = np.array([1, 2, 5])
        Output:
            np.array([[1, 0,  0, 1],
                    [0, 0, -1, 2],
                    [0, 1,  0, 5],
                    [0, 0,  0, 1]])
        """
        f = [0,0,0,0,0]
        f_prime = [0,0,0,0,0]
        f_primeprime = [0,0,0,0,0]
        R_current, p_current = mr.TransToRp(M_current)

        for i in range(3):
            dim = intToDim(i)
            f[4] += self.betas[dim][2]**2
            f[3] += 2*self.betas[dim][2]*self.betas[dim][1]
            f[2] += 2*self.betas[dim][2]*(self.betas[dim][0]-p_current[i]) + self.betas[dim][1]**2
            f[1] += 2*self.betas[dim][1]*(self.betas[dim][0]-p_current[i])
            f[0] += (self.betas[dim][0]-p_current[i])**2

        # Newton Raphson
        possible, collision_time, new_estimates = self.NRM_Mininum_Dist_btwn_Traj_Point(f, 0, 0.001)
        if possible:
            point = [self.getCoordFromTime("x", collision_time), \
                     self.getCoordFromTime("y", collision_time), \
                     self.getCoordFromTime("z", collision_time)]
            # return (True, point, collision_time - self.times[-1])
            
            return (True, point, collision_time - self.times[-1])
        else:
            return (False, None, None)

    # f is the coefficients of the 4th polynomial under the square root
    def NRM_Mininum_Dist_btwn_Traj_Point(self, f, estimate, threshold):
        """ Newton Raphson to solve the quartic equation in the drive
        :param f: polynomial coefficients for the equation 0,1,2,3,4
        :param estimate: where to start
        :return: success, the time from now that the intersection occurs, a list of estimates for troubleshooting
        Example Input:
            R = np.array([[1, 0,  0],
                        [0, 0, -1],
                        [0, 1,  0]])
            p = np.array([1, 2, 5])
        Output:
            np.array([[1, 0,  0, 1],
                    [0, 0, -1, 2],
                    [0, 1,  0, 5],
                    [0, 0,  0, 1]])
        """
        f_prime = [f[1],2*f[2],3*f[3],4*f[4],0]
        f_prime2 =[f_prime[1], 2*f_prime[2], 3*f_prime[3], 0, 0]
        counter = 0
        t = estimate
        new_estimates = []
        while counter < 10:
            dist_prime = 0.5*(f[0] + f[1]*t + f[2]*(t**2) + f[3]*(t**3) + f[4]*(t**4))**(-0.5)* \
                         (f_prime[0] + f_prime[1]*t + f_prime[2]*t**2 + f_prime[3]*t**3)
            new_estimates.append(t)
            if abs(dist_prime) <= threshold: 
                return (True, t, new_estimates)
            dist_prime2 = -0.25*(f[0] + f[1]*t + f[2]*(t**2) + f[3]*(t**3) + f[4]*(t**4))**(-1.5) * \
                          (f_prime[0] + f_prime[1]*t + f_prime[2]*t**2 + f_prime[3]*t**3) + \
                          0.5*(f[0] + f[1]*t + f[2]*(t**2) + f[3]*(t**3) + f[4]*(t**4))**(-0.5) * \
                          (f_prime2[0] + f_prime2[1]*t + f_prime2[2]*t**2 + f_prime2[3]*t**3)
            t = t - dist_prime/dist_prime2
            counter += 1

        return (False, None, None)

    def checkSphereIntersection(self, center, radius):
        # plugging in the equation of the parabola into mag(x - center) = radius^2
        a = [0,0,0,0,0]
        for i in range(3):
            dim = intToDim(i)
            a[4] += self.betas[dim][2]**2
            a[3] += 2*self.betas[dim][2]*self.betas[dim][1]
            a[2] += 2*self.betas[dim][2]*self.betas[dim][0] + self.betas[dim][1]**2 - 2*center[i]*self.betas[dim][2]
            a[1] += 2*self.betas[dim][1]*self.betas[dim][0] - 2*center[i]*self.betas[dim][1]
            a[0] += self.betas[dim][0]**2 - 2*center[i]*self.betas[dim][0] + center[i]**2
        a[0] +=  -1*(radius**2)

        # now using the quartic formula (Ax^4 + Bx^3 +Cx^2 + Dx + E = 0)
        # look into Bairstow's Method, or use Newton Raphson

        # using Netwon Raphson, good estimate for first collision is time0, will always be closest to the first intersection
        possible, collision_time, new_estimates = self.newtonRaphsonQuartic(a, 0, 0.001)

        if possible:
            # plug in collision_time into trajectory and record the x,y,z            
            fig = plt.figure(figsize=plt.figaspect(0.5))
            ax = fig.add_subplot(1,2,1, projection="3d")
            ax.set_xlabel("X")
            ax.set_ylabel("Y")
            ax.set_zlabel("Z")
            ax.axes.set_xlim3d(left=-2, right=2)
            ax.axes.set_ylim3d(bottom=-2, top=2)
            ax.axes.set_zlim3d(bottom=-2, top=2)
            ax.scatter(0,0,0.180212)
            ax.text(0,0,0.180212, "center")
            for i in range(len(new_estimates)):
                point = [self.getCoordFromTime("x", new_estimates[i]), \
                         self.getCoordFromTime("y", new_estimates[i]), \
                         self.getCoordFromTime("z", new_estimates[i])]
                ax.scatter(point[0], -1*point[2], point[1])
                ax.text(point[0], -1*point[2], point[1], i)

            point = [self.getCoordFromTime("x", collision_time), \
                     self.getCoordFromTime("y", collision_time), \
                     self.getCoordFromTime("z", collision_time)]
            ax.scatter(point[0], -1*point[2], point[1])
            ax.text(point[0], -1*point[2], point[1], "final_point")
            
            u,v = np.mgrid[0:2*np.pi:40j, 0:np.pi:20j]
            x = np.cos(u)*np.sin(v)*0.4087037
            y = np.sin(u)*np.sin(v)*0.4087037 + 0.180212
            z = np.cos(v)*0.4087037
            ax.plot_wireframe(x, -1*z, y, color="y")

            t = np.arange(0,2,0.01)
            x_traj = self.betas["x"][0] + self.betas["x"][1]*t + self.betas["x"][2]*(t**2)
            y_traj = self.betas["y"][0] + self.betas["y"][1]*t + self.betas["y"][2]*(t**2)
            z_traj = self.betas["z"][0] + self.betas["z"][1]*t + self.betas["z"][2]*(t**2)
            ax.plot3D(x_traj, -1*z_traj, y_traj, color="r")

            bx = fig.add_subplot(1,2,2)
            bx.set_ylim([-1,1])
            distance_to_zero = a[0] + a[1]*t + a[2]*(t**2) + a[3]*(t**3) + a[4]*(t**4)
            bx.plot(t,distance_to_zero)
            bx.plot(t,0*t)
            for i in range(len(new_estimates)):
                distance = a[0] + a[1]*new_estimates[i] + a[2]*(new_estimates[i]**2) + a[3]*(new_estimates[i]**3) + a[4]*(new_estimates[i]**4)
                bx.scatter(new_estimates[i], distance)
                bx.text(new_estimates[i], distance, i)

            plt.show()

            return True, point, collision_time - self.times[-1]
        else:
            # if false, return to cobra in 5 seconds
            return False, [0.201582, 0.498462, 0], 5


    # problem with newton Raphson is there is a max in the parabola before the intersection, but distance from the center shouldn't be a problem
    def newtonRaphsonQuartic(self, a, estimate, threshold):
        counter = 0
        t = estimate
        new_estimates = []
        while counter < 10:
            f = a[0] + a[1]*t + a[2]*(t**2) + a[3]*(t**3) + a[4]*(t**4)
            new_estimates.append(t)
            if abs(f) <= threshold: 
                return True, t, new_estimates
            f_prime = a[1] + 2*a[2]*(t) + 3*a[3]*(t**2) + 4*a[4]*(t**3)
            t = t - f/f_prime
            counter += 1
        return False, estimate, new_estimates



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
        predicted_x_low = self.getPredictedPosition(self.betas_low["x"], time_delta)
        predicted_x_high = self.getPredictedPosition(self.betas_high["x"], time_delta)
        predicted_y_low = self.getPredictedPosition(self.betas_low["y"], time_delta)
        predicted_y_high = self.getPredictedPosition(self.betas_high["y"], time_delta)
        predicted_z_low = self.getPredictedPosition(self.betas_low["z"], time_delta)
        predicted_z_high = self.getPredictedPosition(self.betas_high["z"], time_delta)
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

"""
    # for in plane coord errors, couldn't really think of a good value: some of the error will come from the image being off, some will come from deproject_pixel_to_point, some from canny
    # ended up going for half the size of the ball as total error (20mm), so 10mm on each side
    # for time errors, couldn't find anything again, so just a random value ( i get a sample timestamp accuracy on p72, but that is for the imu)
"""