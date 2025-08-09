import serial

class UnitV:
    def __init__(self,port="/dev/ttyUSB1"):
        self.ser = serial.Serial(port, 115200, timeout=1)
        self.ser.flush()
        self.header = b'\0x00\0x01\0x02\0x03'
        self.footer = b'\0xFF\0xFF\0xFF\0xFF'
        self.greens = []
        self.reds = []
    
    def update_data(self):
        if self.ser.in_waiting == 0:
            return False
        while self.ser.in_waiting > 1:
            self.ser.readline()
        line = self.ser.readline().strip()
        if line.startswith(self.header) and line.endswith(self.footer):
            data = line[len(self.header):-len(self.footer)]
            len_green = data[0]
            len_red = data[1]
            self.greens = data[2:2+len_green]
            self.reds = data[2+len_green:2+len_green+len_red]
            return True
        return False
