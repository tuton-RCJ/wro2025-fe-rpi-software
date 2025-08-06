#!/usr/bin/env python
#
# *********     Gen Write Example      *********
#
#
# Available SCServo model on this example : All models using Protocol SCS
# This example is tested with a SCServo(STS/SMS), and an URT
#

import sys
import os
import time
sys.path.append('/home/tuton/wro2025-fe-rpi-software/package')
from STS3032 import sts3032
sts = sts3032()
sts.drive(50, 30)  
time.sleep(2)
sts.stop() 
sts.close_port() 