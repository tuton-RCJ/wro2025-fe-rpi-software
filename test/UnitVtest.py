import sys
sys.path.append('/home/tuton/wro2025-fe-rpi-software/package')
from UnitV import unitv
UV = unitv()
try:
    while True:
        print(*UV.update_data())
finally:
    UV.close()