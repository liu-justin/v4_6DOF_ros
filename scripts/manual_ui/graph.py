import tkinter as tk
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import (
                                    FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

from modules import modern_robotics as mr
from modules import unpack as unp
import os
import rospy
import std_msgs.msg as msg

class TransfGraph(tk.Frame):
    def __init__(self, master, motors):
        super().__init__(master)

        cwd = os.getcwd()
        print(cwd)

        self.M_rest, self.T_list, self.body_list, self.G_list = unp.unpack_XML("/home/justin/catkin_ws/src/v4_6dof/scripts/6DoF_URDF.xml")

        self.pos_list = [0,0,0,0,0,0]

        self.subR1 = rospy.Subscriber('pos_motorR1', msg.Float32,self.updatePosThruR1)
        self.subT1 = rospy.Subscriber('pos_motorT1', msg.Float32,self.updatePosThruT1)
        self.subT2 = rospy.Subscriber('pos_motorT2', msg.Float32,self.updatePosThruT2)
        self.subR2 = rospy.Subscriber('pos_motorR2', msg.Float32,self.updatePosThruR2)
        self.subT3 = rospy.Subscriber('pos_motorT3', msg.Float32,self.updatePosThruT3)
        self.subR3 = rospy.Subscriber('pos_motorR3', msg.Float32,self.updatePosThruR3)
        # self.eteta_home = np.array([0,-1*np.pi/2, np.pi/2,0,0,0])
        # self.M_home = mr.FKinBody(self.M_rest, self.body_list, self.theta_home)

    def createWidgets(self):
        self.fig = Figure(figsize=(4, 4), dpi=100)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)  # A tk.DrawingArea.
        self.canvas.draw()

        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.axes.set_xlim3d(left=-2, right=2)
        self.ax.axes.set_ylim3d(bottom=-2, top=2)
        self.ax.axes.set_zlim3d(bottom=-1, top=1)
        self.x_norm, = self.ax.plot([0,0],[0,0],[0,0])
        self.y_norm, = self.ax.plot([0,0],[0,0],[0,0])
        self.z_norm, = self.ax.plot([0,0],[0,0],[0,0])
        self.plotTransf(self.M_rest)

        # toolbar is not helpful now
        # self.toolbar = NavigationToolbar2Tk(self.canvas, self)
        # self.toolbar.update()
        self.canvas.get_tk_widget().pack()

    def updatePosThruR1(self,data):
        self.pos_list[0] = data.data
        self.updatePlot()

    def updatePosThruT1(self,data):
        self.pos_list[1] = data.data
        self.updatePlot()

    def updatePosThruR2(self,data):
        self.pos_list[2] = data.data
        self.updatePlot()

    def updatePosThruT2(self,data):
        self.pos_list[3] = data.data
        self.updatePlot()

    def updatePosThruR3(self,data):
        self.pos_list[4] = data.data
        self.updatePlot()

    def updatePosThruT3(self,data):
        self.pos_list[5] = data.data
        self.updatePlot()


    def updatePlot(self):
        # rospy.loginfo(f"changing the plot with {self.pos_list}")
        new_M = mr.FKinBody(self.M_rest, self.body_list, np.array(self.pos_list))
        print(new_M)
        self.plotTransf(new_M)


    def plotTransf(self, M):
        
        R,p = mr.TransToRp(M)
        x_arrow_end = p + 0.2*R[:,0]
        y_arrow_end = p + 0.2*R[:,1]
        z_arrow_end = p + 0.2*R[:,2]
        x_normal = np.c_[p,x_arrow_end]
        y_normal = np.c_[p,y_arrow_end]
        z_normal = np.c_[p,z_arrow_end]
        
        self.x_norm.set_xdata(x_normal[0])
        self.x_norm.set_ydata(x_normal[1])
        self.x_norm.set_3d_properties(x_normal[2])

        self.y_norm.set_xdata(y_normal[0])
        self.y_norm.set_ydata(y_normal[1])
        self.y_norm.set_3d_properties(y_normal[2])

        self.z_norm.set_xdata(z_normal[0])
        self.z_norm.set_ydata(z_normal[1])
        self.z_norm.set_3d_properties(z_normal[2])
        # self.fig.show()
        # plt.show()
        plt.draw()

        # self.ax.plot(x_norm[0],x_norm[1],x_norm[2])
        # self.ax.plot(y_norm[0],y_norm[1],y_norm[2])
        # self.ax.plot(z_norm[0],z_norm[1],z_norm[2])

