from scservo_sdk import *
import time 
class sts3032:
    def __init__(self, port="/dev/ttyAMA5", front_servo_id=3, back_servo_id=2, center_degree=2048, default_speed=50):
        self.portHandler = PortHandler(port)
        self.packetHandler = sms_sts(self.portHandler)
        self.front_servo_id = front_servo_id
        self.back_servo_id = back_servo_id
        self.center_degree = center_degree
        self.default_speed = default_speed
        self.max_deg = 65
        self._init_port()
    
    def _init_port(self):
        flag = True
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            flag = False

        if self.portHandler.setBaudRate(1000000):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            flag = False
            
        scs_model_number, scs_comm_result, scs_error = self.packetHandler.ping(self.front_servo_id)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
            flag = False
        else:
            print("[ID:%03d] ping to front servo Succeeded. SCServo model number : %d" % (self.front_servo_id, scs_model_number))

        if scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))
        self.packetHandler.ServoMode(self.front_servo_id)
        print("Front servo is set to servo mode")
        self.packetHandler.WritePosEx(self.front_servo_id, self.center_degree, 0, 0)
        time.sleep(1)
            
        scs_model_number, scs_comm_result, scs_error = self.packetHandler.ping(self.back_servo_id)
        if scs_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(scs_comm_result))
            flag = False
        else:
            print("[ID:%03d] ping to back servo Succeeded. SCServo model number : %d" % (self.back_servo_id, scs_model_number))
        if scs_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(scs_error))
        self.packetHandler.WheelMode(self.back_servo_id)
        print("Back servo is set to wheel mode.")
        
        print("STS3032 initialized successfully" if flag else "STS3032 initialization failed.")

    def stop(self):
        self.packetHandler.WriteSpec(self.back_servo_id, 0, 0)

    def drive(self, speed=-1, degree=-1):
        """[summary]
        Args:
            speed (int): Speed of the back servo. Range is 1 to 100.
            degree (float): Degree of the front servo. Degrees is restricted to -30° to 30°. 
        """
        self.stop()
        time.sleep(0.01)
        if speed == -1:
            speed = self.default_speed
        else:
            speed = max(1, min(speed, 100))
        if degree == -1:
            degree = 0
        else:
            degree = max(-self.max_deg, min(degree, self.max_deg))
        speed *= 75
        degree = int(degree/360 * 4096 + self.center_degree)
        self.packetHandler.WritePosEx(self.front_servo_id, degree, speed, 0)
        print(degree, speed, degree/speed)
        time.sleep(degree/speed + 0.01) # Adjust sleep time to ensure the command is processed
        self.packetHandler.WriteSpec(self.back_servo_id, speed, 0)
    
    def turn_left(self,speed=-1,angle=65):
        if speed == -1:
            speed = self.default_speed
        angle = min(angle, self.max_deg)
        target_degree = self.center_degree + int(angle/360 * 4096)
        self.packetHandler.WritePosEx(self.front_servo_id, target_degree, speed * 75, 0)
        time.sleep(0.1)
        self.packetHandler.WriteSpec(self.back_servo_id, speed * 75, 0)
        time.sleep(1)
        self.packetHandler.WriteSpec(self.back_servo_id, 0, 0)
        time.sleep(0.1)
        self.packetHandler.WritePosEx(self.front_servo_id, self.center_degree, self.default_speed * 75, 0)
        time.sleep(0.1)

    def turn_right(self, speed=-1, angle=65):
        if speed == -1:
            speed = self.default_speed
        angle = min(angle, self.max_deg)
        target_degree = self.center_degree - int(angle/360 * 4096)
        self.packetHandler.WritePosEx(self.front_servo_id, target_degree, speed * 75, 0)
        time.sleep(0.1)
        self.packetHandler.WriteSpec(self.back_servo_id, speed * 75, 0)
        time.sleep(1)
        self.packetHandler.WriteSpec(self.back_servo_id, 0, 0)
        time.sleep(0.1)
        self.packetHandler.WritePosEx(self.front_servo_id, self.center_degree, self.default_speed * 75, 0)
        time.sleep(0.1)


    def close_port(self):
        self.portHandler.closePort()
        print("Port closed successfully.")
