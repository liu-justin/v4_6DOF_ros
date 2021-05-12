import tkinter as tk
# SPEEDS = [1,2,5,10,20]
SPEEDS = [0.1,0.2,0.5,1,2]

class SpeedSelector(tk.Frame):
    def __init__(self, master):
        super().__init__(master)

    def createWidgets(self):
        self.label = tk.Label(master=self, text="speed (rad/s)")
        
        self.speed = tk.StringVar(self)
        self.speed.set(SPEEDS[0])
        self.speedDropdown = tk.OptionMenu(self, self.speed, *SPEEDS)

        self.confirm = tk.Button(master=self, text="ok", command=self.updateSpeed)

    def layout(self):
        self.rowconfigure(0,minsize=100, weight=1)
        self.columnconfigure([0,1,2], minsize=100, weight=1)
        self.label.grid(row=0, column=0)
        self.speedDropdown.grid(row=0, column=1)
        self.confirm.grid(row=0,column=2)

    def updateSpeed(self):
                self.label["text"]=f"speed is {self.speed.get()} rad/s"

    def getSpeed(self):
        return self.speed.get()