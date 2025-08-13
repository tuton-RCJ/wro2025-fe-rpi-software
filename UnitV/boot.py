
import sensor, image, time, lcd
from machine import UART

from fpioa_manager import fm
import ustruct
lcd.init(freq=15000000)
sensor.reset()
sensor.skip_frames(time=2000)
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QQVGA)
sensor.set_jb_quality(85)
sensor.skip_frames(time = 2000)
sensor.set_auto_exposure(False,exposure_us=10000)
sensor.skip_frames(time = 2000)
clock = time.clock()
thre_red = [(20, 75, 40, 60, 30, 60)]
thre_green = [(30,100, -35, -15, -30, -10)]
colors = [
    (255, 0, 0),
    (0, 255, 0),
    (255, 255, 0),
    (0, 0, 255),
    (255, 0, 255),
    (0, 255, 255),
    (255, 255, 255),
]

fm.register(35, fm.fpioa.UART1_TX, force=True)
fm.register(34, fm.fpioa.UART1_RX, force=True)
uart = UART(UART.UART1, 115200,8,0,0, timeout=1000, read_buf_len=4096)


def send(value):
    uart.write(ustruct.pack('B',value))

while(True):
    clock.tick()
    img = sensor.snapshot()
    green = []
    red = []
    max_red = 0
    max_rect = None
    for obj in img.find_blobs(thre_green,area_threshold=300,merge=True,x_stride=1):
        img.draw_rectangle(obj[:4],color=colors[5])
        print(obj[2],obj[3])
        if obj[3] / obj[2] > 1.3 and obj[3] > 25:
            max_rect = obj if max_rect is None or obj[2] * obj[3] > max_rect[2] * max_rect[3] else max_rect
    if not max_rect is None:
        green.append((max_rect[0]+max_rect[2]//2,max_rect[1]+max_rect[3]//2))

    max_green = 0
    max_rect = None
    for obj in img.find_blobs(thre_red,area_threshold=300,merge=True,x_stride=1,margin=40):
        img.draw_rectangle(obj[:4],color=colors[5])
        print(obj[2],obj[3])
        if obj[3] / obj[2] > 1.3 and obj[3] > 25:
            max_rect = obj if max_rect is None or obj[2] * obj[3] > max_rect[2] * max_rect[3] else max_rect
    if not max_rect is None:
        red.append((max_rect[0]+max_rect[2]//2,max_rect[1]+max_rect[3]//2))
    if len(red) == 1:
        send(red[0][0])
        send(red[0][1])
    else:
        send(255)
        send(255)
    if len(green) == 1:
        send(green[0][0])
        send(green[0][1])
    else:
        send(255)
        send(255)
    uart.write(b"\n")
    print(clock.fps(),sensor.get_exposure_us(),sensor.get_gain_db())
