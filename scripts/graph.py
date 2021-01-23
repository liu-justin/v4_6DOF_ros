import tkinter as tk
import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import (
                                    FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure

class TransfGraph(tk.Frame):
    def __init__(self, master, motors):
        super().__init__(master)

        # self.wm_title("Embedding in Tk")

        # canvas.grid(row=1,column=5)
        # self.pack()
    def createWidgets(self):
        self.fig = Figure(figsize=(4, 4), dpi=100)

        self.canvas = FigureCanvasTkAgg(self.fig, master=self)  # A tk.DrawingArea.
        self.canvas.draw()

        self.ax = self.fig.add_subplot(111, projection="3d")
        t = np.arange(0, 3, .01)
        self.ax.plot(t, 2 * np.sin(2 * np.pi * t))

        # self.toolbar = NavigationToolbar2Tk(self.canvas, self)
        # self.toolbar.update()
        self.canvas.get_tk_widget().pack()

    # def layout(self):
        # self.canvas.get_tk_widget().grid(row=0, column=0)
