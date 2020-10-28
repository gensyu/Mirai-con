import serial
import time
import construct as cs

def cal_checksum(data):
    c_sum = 0 
    for byte_data in data:
        c_sum = (c_sum + byte_data) % 256
    return c_sum.to_bytes(1, 'big')

sendfmt = cs.BitStruct(
    "header" / cs.Bytewise(cs.Const(b"\xAA")),
    "rw" / cs.BitsInteger(1),
    "addr" / cs.BitsInteger(7),
    "wdata" / cs.BitsInteger(16),
)

recvfmt = cs.BitStruct(
    "header" / cs.Bytewise(cs.Const(b'\xAA')),
    "pad" / cs.Padding(1),
    "addr" / cs.BitsInteger(7),
    "rdata" / cs.BitsInteger(16),
    "line_0" / cs.Flag,
    "line_1" / cs.Flag,
    "line_2" / cs.Flag,
    "line_3" / cs.Flag,
    "line_4" / cs.Flag,
    "line_5" / cs.Flag,
    "line_6" / cs.Flag,
    "line_7" / cs.Flag,
    "tof_f" / cs.BitsInteger(16),
    "tof_r" / cs.BitsInteger(16),
    "tof_l" / cs.BitsInteger(16),
    "tof_b" / cs.BitsInteger(16),
    "reserved" / cs.Bytewise(cs.Const(b'\x00\x00')),
    "sum" / cs.BitsInteger(8)
)

s = serial.Serial(port="/dev/ttyS0", baudrate=115200)

send_data = sendfmt.build({
    "rw" : 0x1,
    "addr" : 0x00,
    "wdata" : 0x0000,
})

send_data = send_data + cal_checksum(send_data)
print(send_data)

while  True:

    s.write(send_data)
    time.sleep(20e-3)
    data = s.read_all()
    recv_data =  recvfmt.parse(data)
    print(recv_data)
    time.sleep(20e-3)

