         
import sensor, image, time, lcd
from machine import UART
import ustruct
lcd.init(freq=15000000)
sensor.reset()        
sensor.set_auto_gain(False,0)
sensor.skip_frames(time=2000)
sensor.set_pixformat(sensor.RGB565) 
sensor.set_framesize(sensor.QQVGA) 
sensor.set_jb_quality(85)
sensor.skip_frames(time = 2000)
sensor.set_auto_exposure(False,exposure_us=10000)
sensor.skip_frames(time = 2000)
clock = time.clock()              
thre_red = [(20, 75, 40, 60, 30, 60)]
thre_green = [(30,100, -60, -25, -5, 20)]
colors = [
    (255, 0, 0),
    (0, 255, 0),
    (255, 255, 0),
    (0, 0, 255),
    (255, 0, 255),
    (0, 255, 255),
    (255, 255, 255),
]
uart = UART(UART.UART1, 115200,8,0,0, timeout=1000, read_buf_len=4096)

mod = 1 >> 8

def bit_tuple(x,y):
    return (x << 8) + y
def bit_tie(x):
    return x >> 8, x % mod

while(True):
    clock.tick()       
    while sensor.get_auto_gain() != 0.0:
        sensor.set_auto_gain(False,0,0)
    img = sensor.snapshot()  
    green = []
    red = []
    
    for obj in img.find_blobs(thre_green,area_threshold=300,merge=True,x_stride=1):			
        if obj[3] / obj[2] > 1.4 and obj[2] > 50:
            img.draw_rectangle(obj[:4],color=colors[5])
            green.append(bit_tuple(obj[0]+obj[2]//2,obj[1]+obj[3]//2))
            
    for obj in img.find_blobs(thre_red,area_threshold=300,merge=True,x_stride=1,margin=40):
        if obj[2] / obj[3] > 1.4 and obj[2] > 50:
            img.draw_rectangle(obj[:4],color=colors[5])
            bit_tuple(obj[0]+obj[2]//2,obj[1]+obj[3]//2)
            red.append(bit_tuple(obj[0]+obj[2]//2,obj[1]+obj[3]//2))

    header = bytearray([0x00, 0x01, 0x02, 0x03,len(green), len(red)])
    footer = bytearray([0xFF, 0xFF, 0xFF, 0xFF])
    data = bytearray()
    data.extend(header)
    data.extend(green)
    data.extend(red)
    data.extend(footer)
    uart.write(data)
    uart.flush()
    
    print(clock.fps(),sensor.get_exposure_us(),sensor.get_gain_db())                                        