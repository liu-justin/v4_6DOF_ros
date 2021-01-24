import tkinter as tk
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import (
                                    FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure

import modern_robotics as mr
import unpack as unp
import os

class TransfGraph(tk.Frame):
    def __init__(self, master, motors):
        super().__init__(master)

        cwd = os.getcwd()
        print(cwd)

        self.T_ee, self.T_list, self.body_list, self.G_list = unp.unpack_XML("/home/justin/catkin_ws/src/v4_6dof/scripts/6DoF_URDF.xml")

        self.theta_home = np.array([0,-1*np.pi/2, np.pi/2,0,0,0])
        self.M_rest = self.T_ee
        self.M_home = mr.FKinBody(self.T_ee, self.body_list, self.theta_home)

    def createWidgets(self):
        self.fig = Figure(figsize=(4, 4), dpi=100)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)  # A tk.DrawingArea.
        self.canvas.draw()

        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.axes.set_xlim3d(left=-2, right=2)
        self.ax.axes.set_ylim3d(bottom=-2, top=2)
        self.ax.axes.set_zlim3d(bottom=-1, top=1)
        self.plotTransf(self.M_rest)

        # toolbar is not helpful now
        # self.toolbar = NavigationToolbar2Tk(self.canvas, self)
        # self.toolbar.update()
        self.canvas.get_tk_widget().pack()

    def plotTransf(self, M):
        R,p = mr.TransToRp(M)
        x_arrow_end = p + 0.2*R[:,0]
        y_arrow_end = p + 0.2*R[:,1]
        z_arrow_end = p + 0.2*R[:,2]
        x_norm = np.c_[p,x_arrow_end]
        y_norm = np.c_[p,y_arrow_end]
        z_norm = np.c_[p,z_arrow_end]

        self.ax.plot(x_norm[0],x_norm[1],x_norm[2])
        self.ax.plot(y_norm[0],y_norm[1],y_norm[2])
        self.ax.plot(z_norm[0],z_norm[1],z_norm[2])

