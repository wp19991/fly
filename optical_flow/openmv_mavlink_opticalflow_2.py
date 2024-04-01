# 通过mavlink实现光流
#
# 该脚本使用MAVLink协议向ArduPilot / PixHawk控制器发送光流检测，以使用您的OpenMV Cam进行位置控制。
#
# P4 = TXD

import sensor
import struct
import time
import machine

UART_BAUDRATE = 115200
MAV_system_id = 1
MAV_component_id = 0x54
packet_sequence = 0

# 低于0.1左右（YMMV），结果只是噪音
MAV_OPTICAL_FLOW_confidence_threshold = (0.1)

# LED 控制
led = machine.LED("LED_BLUE")
led_state = 0


def update_led():
    global led_state
    led_state = led_state + 1
    if led_state == 10:
        led.on()
    elif led_state >= 20:
        led.off()
        led_state = 0


# 链接设置
uart = machine.UART(3, UART_BAUDRATE, timeout_char=1000)


# https://github.com/mavlink/c_library_v1/blob/master/checksum.h
def checksum(data, extra):
    output = 0xFFFF
    for i in range(len(data)):
        tmp = data[i] ^ (output & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    tmp = extra ^ (output & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return output


MAV_OPTICAL_FLOW_message_id = 100
MAV_OPTICAL_FLOW_id = 0  # unused
MAV_OPTICAL_FLOW_extra_crc = 175


# http://mavlink.org/messages/common#OPTICAL_FLOW
# https://github.com/mavlink/c_library_v1/blob/master/common/mavlink_msg_optical_flow.h
def send_optical_flow_packet(x, y, c):
    global packet_sequence
    temp = struct.pack(
        "<qfffhhbb", 0, 0, 0, 0, int(x), int(y), MAV_OPTICAL_FLOW_id, int(c * 255)
    )
    temp = struct.pack(
        "<bbbbb26s",
        26,
        packet_sequence & 0xFF,
        MAV_system_id,
        MAV_component_id,
        MAV_OPTICAL_FLOW_message_id,
        temp,
    )
    temp = struct.pack("<b31sh", 0xFE, temp, checksum(temp, MAV_OPTICAL_FLOW_extra_crc))
    packet_sequence += 1
    uart.write(temp)
    update_led()


sensor.reset()  # 复位并初始化传感器sensor.
sensor.set_pixformat(sensor.RGB565)  # 设置图像色彩格式，有RGB565色彩图和GRAYSCALE灰度图两种
sensor.set_framesize(sensor.B64X32)  # 设置图像大小为64x32…(或64 x64)…
sensor.skip_frames(time=2000)  # 等待设置生效。
clock = time.clock()  # 创建一个时钟对象来跟踪FPS帧率。

# 从主帧缓冲区的RAM中取出以分配第二帧缓冲区。
# 帧缓冲区中的RAM比MicroPython堆中的RAM多得多。
# 但是，在执行此操作后，您的某些算法的RAM会少得多......
# 所以，请注意现在摆脱RAM问题要容易得多。
extra_fb = sensor.alloc_extra_fb(sensor.width(), sensor.height(), sensor.RGB565)
extra_fb.replace(sensor.snapshot())

while True:
    clock.tick()  # 追踪两个snapshots()之间经过的毫秒数.
    img = sensor.snapshot()  # 拍一张照片，返回图像

    displacement = extra_fb.find_displacement(img)
    extra_fb.replace(img)

    # 没有滤波，偏移结果是嘈杂的，所以我们降低了一些精度
    sub_pixel_x = int(-displacement.x_translation() * 35)
    sub_pixel_y = int(displacement.y_translation() * 53)

    send_optical_flow_packet(sub_pixel_x, sub_pixel_y, displacement.response())

    print(
        "{0:+f}x {1:+f}y {2} {3} FPS".format(
            sub_pixel_x, sub_pixel_y, displacement.response(), clock.fps()
        )
    )
