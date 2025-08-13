import serial

class unitv:
    def __init__(self,port="/dev/ttyAMA2"):
        self.ser = serial.Serial(port, 115200, timeout=1)
        self.ser.flush()
        self.greens = []
        self.reds = []
    
    def update_data(self):
        if self.ser.in_waiting == 0:
            while self.ser.in_waiting < 0:
                pass
        while self.ser.in_waiting > 1:
            self.ser.readline()
        line = list(map(int,self.ser.readline().strip()))
        return line
    
    def close(self):
        self.ser.close()