import tkinter as tk
SPEEDS = [1,2,5,10,20]
class SpeedSelector(tk.Frame):
    def __init__(self, master):
        super().__init__(master)

        # self.frame = tk.Frame()
        self.rowconfigure(0, minsize=100, weight=1)
        self.columnconfigure([0, 1, 2], minsize=100, weight=1)
        self.label = tk.Label(master=self, text="speed (rad/s)")
        self.label.grid(row=0, column=0)

        self.speed = tk.StringVar(self)
        self.speed.set(SPEEDS[0])
        self.speedDropdown = tk.OptionMenu(self, self.speed, *SPEEDS)
        self.speedDropdown.grid(row=0, column=1)

        # def updateSpeed():
        #     label["text"]=f"speed is {self.speed.get()} rad/s"

        speedConfirm = tk.Button(master=self, text="ok", command=self.updateSpeed)
        speedConfirm.grid(row=0,column=2)

        self.pack()

    def updateSpeed(self):
                self.label["text"]=f"speed is {self.speed.get()} rad/s"

    def getSpeed(self):
        return self.speed.get()