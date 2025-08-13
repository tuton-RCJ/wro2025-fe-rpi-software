import serial

class UnitV:
    def __init__(self,port="/dev/ttyUSB1"):
        self.ser = serial.Serial(port, 115200, timeout=1)
        self.ser.flush()
        self.greens = []
        self.reds = []
    
    def update_data(self):
        # 赤の中心座標(X), 緑の重心座標(X)
        if self.ser.in_waiting == 0:
            while self.ser.in_waiting == 0:
                pass
        while self.ser.in_waiting > 1:
            self.ser.readline()
        ob = list(map(int,self.ser.readline().strip()))
        return ob[0], ob[2]
    
    def is_empty(self):
        ud = self.update_data()
        return all(ud, lambda x: x == 255)
