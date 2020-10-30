import serial
import time
import construct as cs
from enum import Enum

class MOTORDIR(Enum):
    CW = 0x01
    CCW = 0x00



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
    "mag_x" / cs.BitsInteger(16),
    "mag_y" / cs.BitsInteger(16),
    "mag_z" / cs.BitsInteger(16),
    "acc_x" / cs.BitsInteger(16),
    "acc_y" / cs.BitsInteger(16),
    "acc_z" / cs.BitsInteger(16),
    # "reserved" / cs.Bytewise(cs.Const(b'\x00\x00')),
    "sum" / cs.BitsInteger(8)
)

def cal_checksum(data):
    c_sum = 0 
    for byte_data in data:
        c_sum = (c_sum + byte_data) % 256
    return c_sum.to_bytes(1, 'big')

class CarDeviceComm:
    def __init__(self, port="/dev/ttyS0", baudrate=500000) -> None:
        self._serial = serial.Serial(port=port, baudrate=baudrate)
        self.software_reset()
    
    def _sendframe(self, rw, addr, wdata,):
        sendframe = sendfmt.build(dict(rw=rw, addr=addr, wdata=wdata))
        sendframe = sendframe + cal_checksum(sendframe)
        return self._serial.write(sendframe)
    
    def _recv_frame(self):
        recvframe = self._serial.read_all()
        # print(recvframe)
        return recvfmt.parse(recvframe)

    def _query_frame(self, rw, addr, wdata, wait_time=20e-3):
        self._sendframe(rw, addr, wdata)
        time.sleep(wait_time)
        return self._recv_frame()

    def get_sensordata(self) -> dict:
        """各種センサデータ取得

        Returns
        -------
        dict
            [description]
        """
        return self._query_frame(rw=0x1, addr=0x00, wdata=0x0000)

    def software_reset(self):
        """ソフトウェアリセット実行 0x00
        """
        self._write_frame(rw=0x0, addr=0x00, wdata=0x5A5A)

    def left_motor(self, direcion: int, duty: float):
        """左モータ制御 0x10

        Parameters
        ----------
        direcion : int
            CW: 1, CCW: 0
        duty : float
            PWM duty
        """
        duty_int = int(duty * (2^8-1))
        if direcion == MOTORDIR.CW:
            wdata = (0x01 << 8) + duty_int
        elif direcion == MOTORDIR.CCW:
            wdata = (0x00 << 8) + duty_int
        else:
            raise
        self._write_frame(rw=0x0, addr=0x1, wdata=wdata)

    def right_motor(self, direcion: int, duty: float):
        """右モータ制御 0x11

        Parameters
        ----------
        direcion : int
            CW: 1, CCW: 0
        duty : float
            PWM duty
        """
        duty_int = int(duty * (2^8-1))
        if direcion == MOTORDIR.CW:
            wdata = (0x01 << 8) + duty_int
        elif direcion == MOTORDIR.CCW:
            wdata = (0x00 << 8) + duty_int
        else:
            raise
        self._write_frame(rw=0x0, addr=0x11, wdata=wdata)

    def change_linetrace_mode(self, mode: int):
        """Auto Line Trace モード変更 0x20

        Parameters
        ----------
        mode : int
            Disable: 0
            Use Line sensor: 1
            Use Line sensor: 2
        """
        pass

    def linetrace_max_speed(self, duty: float):
        """LineTrace モード変更 0x21

        Parameters
        ----------
        duty : float
        """
        pass

    def change_auto_brake(self, mode: int, distance=50):
        """自動ブレーキモード変更 0x22

        Parameters
        ----------
        mode : int
            Disable: 0, enable (by Front TOF): 1
        distance : int, optional
            threshold distance [mm], by default 50 mm
        """
        pass

    def change_linetrace_threshold(self, pattern: int):
        """LineTrace モード変更 0x23

        Parameters
        ----------
        duty : float
        """
        pass

    def change_straight_tof_threshold(
        self, left_distance: float, right_distance: float
    ):
        """LineTrace モード変更 0x24, 0x25

        Parameters
        ----------
        duty : float
        """
        pass

if __name__ == "__main__":
    car_device = CarDeviceComm()
    while True:
        print(car_device.get_sensordata())
        time.sleep(10e-3)