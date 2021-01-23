import tkinter as tk
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import (
                                    FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure

import modern_robotics as mr

class TransfGraph(tk.Frame):
    def __init__(self, master, motors):
        super().__init__(master)

        self.T_ee, self.T_list, self.body_list, self.G_list = unp.unpack_XML("scripts/6DoF_URDF.xml")

        self.theta_home = np.array([0,-1*np.pi/2, np.pi/2,0,0,0])
        self.M_rest = T_ee
        self.M_home = mr.FKinBody(T_ee, body_list, theta_home)

    def createWidgets(self):
        self.fig = Figure(figsize=(4, 4), dpi=100)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)  # A tk.DrawingArea.
        self.canvas.draw()

        self.ax = self.fig.add_subplot(111, projection="3d")
        t = np.arange(0, 3, .01)
        self.ax.plot(t, 2 * np.sin(2 * np.pi * t))

        # toolbar is not helpful now
        # self.toolbar = NavigationToolbar2Tk(self.canvas, self)
        # self.toolbar.update()
        self.canvas.get_tk_widget().pack()

    def plotTransf(self, M):
        R,p = mr.TranstoRp(M)
        self.ax.plot(p[0],p[1],p[2])

    def updateTransf(self):
