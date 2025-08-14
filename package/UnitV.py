import serial

class UnitV:
    def __init__(self,port="/dev/ttyAMA2"):
        self.ser = serial.Serial(port, 115200, timeout=1)
        self.ser.flush()
        self.greens = 0
        self.reds = 0
    
    def update_data(self):
        # 赤の中心座標(X), 緑の重心座標(X)
        if self.ser.in_waiting == 0:
            while self.ser.in_waiting == 0:
                pass
        while self.ser.in_waiting > 1:
            self.ser.readline()
        ob = list(map(int,self.ser.readline().strip()))
        self.greens = ob[0]
        self.reds = ob[2]
        return ob[0], ob[2]
    
    def is_empty(self):
        return all(map(lambda x: x == 255, [self.greens, self.reds]))
