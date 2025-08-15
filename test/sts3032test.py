
import sys
import os
import time
sys.path.append('/home/tuton/wro2025-fe-rpi-software/package')
from STS3032 import sts3032
sts = sts3032()
try:
    while True:
        s, d, t = map(int, input("Enter speed and degree: ").split())
        sts.drive(s, d)
        time.sleep(t)
        sts.stop()
        
except KeyboardInterrupt:
    sts.stop()
finally:
    sts.close_port()