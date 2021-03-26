def linear(x1, x2, t1, t2):
    m = (x2 - x1)/(t2 - t1) # BL2_TR1_m
    b = x1 - m*t1
    return m,b

def poly(x1, x2, t1, t2):
    g = -9.81
    b = (x2-x1)/(t2-t1) - g*(t2+t1)
    c = x1 - g*(t1**2) - b*t1
    return b,c

class Trajectory():
    def __init__(self, time, point):
        self.times = [time]
        self.points = [point]
        self.mx = 5
        self.bx = 0
        self.by = 0
        self.cy = 0
        self.mz = 5
        self.bz = 0

        self.constants_established = False

    def appendFirst(self, new_time, new_point):
        m_x, b_x = linear(self.points[-1][0], new_point[0], self.times[-1], new_time)
        b_y, c_y = poly(self.points[-1][1], new_point[1], self.times[-1], new_time)
        m_z, b_z = linear(self.points[-1][2], new_point[2], self.times[-1], new_time)

        if m_x <= 5 and m_z <= 5:
            self.times.append(new_time)
            self.points.append(new_point)
            self.mx = m_x
            self.bx = b_x
            self.by = b_y
            self.cy = c_y
            self.mz = m_z
            self.bz = b_z
            self.constants_established = True
            
    
    def append(self, new_time, new_point):
        if not self.constants_established:
            self.appendFirst(new_time, new_point)
        else:
            predicted_x = self.mx*new_time + self.bx
            predicted_y = -9.81*(new_time**2) + self.by*new_time + self.cy
            predicted_z = self.mz*new_time + self.bz

            if abs((new_point[0]-predicted_x)/predicted_x) < 0.1 and abs((new_point[1]-predicted_y)/predicted_y) < 0.1 and abs((new_point[2]-predicted_z)/predicted_z) < 0.1:
                self.times.append(new_time)
                self.points.append(new_point)
                return True

            else:
                return False