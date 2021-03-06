import serial
import time
import construct as cs
from enum import Enum

class MOTORDIR(Enum):
    CW = 0x01
    CCW = 0x00

class DRIVINGMODE(Enum):
    Disable: 0
    LINETRACE: 1
    TOF: 2

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
    "mag_x" / cs.BitsInteger(16, signed=True),
    "mag_y" / cs.BitsInteger(16, signed=True),
    "mag_z" / cs.BitsInteger(16, signed=True),
    "acc_x" / cs.BitsInteger(16, signed=True),
    "acc_y" / cs.BitsInteger(16, signed=True),
    "acc_z" / cs.BitsInteger(16, signed=True),
    # "reserved" / cs.Bytewise(cs.Const(b'\x00\x00')),
    "sum" / cs.BitsInteger(8)
)

def cal_checksum(data):
    c_sum = 0 
    for byte_data in data:
        c_sum = (c_sum + byte_data) % 256
    return c_sum.to_bytes(1, 'big')

class CarDevice:
    def __init__(self, port="/dev/ttyS0", baudrate=500000) -> None:
        self._serial = serial.Serial(port=port, baudrate=baudrate)
        self.software_reset()
    
    def _send_frame(self, rw:int, addr:int, wdata:int,):
        """Send UART Frame - MOSI(Raspi→Arduino)

        Args:
            rw (int): 0x1: read, 0x0: write
            addr (int): Register Address[7bit]. ex 0x00
            wdata (int): Write Data[2byte]. 0x0000

        Returns:
            [int]: Number of bytes written.
        """
        sendframe = sendfmt.build(dict(rw=rw, addr=addr, wdata=wdata))
        sendframe = sendframe + cal_checksum(sendframe)
        return self._serial.write(sendframe)
    
    def _recv_frame(self):
        """Recive UART Frame - MISO(Arduino→Raspi)

        Returns:
            [dict like]: Responce Data
        """
        recvframe = self._serial.read_all()
        # print(recvframe)
        return recvfmt.parse(recvframe)

    def _query_frame(self, rw:int, addr:int, wdata:int, wait_time=20e-3):
        """Send UART Frame, after that recive UART frame 

        Args:
            rw (int): 0x1: read, 0x0: write
            addr (int): Register Address[7bit]. ex 0x00
            wdata (int): Write Data[2byte]. 0x0000
            wait_time ([float], optional): times sec. Defaults to 20e-3.

        Returns:
            [dict like]: Responce Data
        """
        self._send_frame(rw, addr, wdata)
        time.sleep(wait_time)
        return self._recv_frame()

    def get_sensordata(self) -> dict:
        """センサデータ取得

        Returns:
            [dict like]: Responce Data
        """
        return self._query_frame(rw=0x1, addr=0x00, wdata=0x0000)

    def software_reset(self):
        """ソフトウェアリセット実行 0x00
        """
        self._send_frame(rw=0x0, addr=0x00, wdata=0x5A5A)
        time.sleep(2)
        self._serial.read_all()

    def vsc3_enable(self):
        """リモコン操縦有効
        """
        self._send_frame(rw=0x0, addr=0x01, wdata=0x01)

    def vsc3_enable(self):
        """リモコン操縦無効
        """
        self._send_frame(rw=0x0, addr=0x01, wdata=0x00)
        

    def left_motor(self, direcion: int, duty: float):
        """左モータ制御 0x10

        Parameters
        ----------
        direcion : int
            CW: 1, CCW: 0
        duty : float
            PWM duty
        """
        duty_int = int(duty * (2**8-1))
        if direcion == MOTORDIR.CW:
            wdata = 0x0100 + duty_int
        elif direcion == MOTORDIR.CCW:
            wdata = 0x0000 + duty_int
        else:
            raise
        self._sendframe(rw=0x0, addr=0x10, wdata=wdata)

    def right_motor(self, direcion: int, duty: float):
        """右モータ制御 0x11

        Parameters
        ----------
        direcion : int
            CW: 1, CCW: 0
        duty : float
            PWM duty
        """
        duty_int = int(duty * (2**8-1))
        if direcion == MOTORDIR.CW:
            wdata = 0x0100 + duty_int
        elif direcion == MOTORDIR.CCW:
            wdata = 0x0000 + duty_int
        else:
            raise
        self._sendframe(rw=0x0, addr=0x11, wdata=wdata)

    def change_adm_mode(self, mode: int):
        """Auto Line Trace モード変更 0x20

        Parameters
        ----------
        mode : int
            Disable: 0
            Use Line sensor: 1
            Use TOF sensor: 2
        """
        wdata = [0x00, 0x10, 0x20][mode]
        self._sendframe(rw=0x0, addr=0x20, wdata=wdata)

    def linetrace_max_speed(self, duty: float):
        """LineTrace モード変更 0x21

        Parameters
        ----------
        duty : float
        """
        duty_int = int(duty * (2^8-1))
        self._sendframe(rw=0x0, addr=0x21, wdata=duty_int)

    def change_auto_brake(self, mode: int, distance: int=50):
        """自動ブレーキモード変更 0x22

        Parameters
        ----------
        mode : int
            Disable: 0, enable (by Front TOF): 1
        distance : int, optional
            threshold distance [mm], by default 50 mm
        """
        if mode == 0:
            wdata = 0x0000
        else:
            wdata = distance
        self._sendframe(rw=0x0, addr=0x21, wdata=wdata)
        

    def change_linetrace_threshold(self, pattern: int):
        """ADM Linetrace しきい値設定 0x23

        Parameters
        ----------
        pattern : int
        """
        pass

    def change_straight_tof_threshold(
        self, left_distance: float, right_distance: float
    ):
        """ADM TOFセンサ しきい値設定 0x24, 0x25

        Parameters
        ----------
        left_distance [mm]: int
        right_distance [mm]: int
        """
        
        self._sendframe(rw=0x0, addr=0x24, wdata=left_distance)
        self._sendframe(rw=0x0, addr=0x25, wdata=right_distance)

    def change_cam_angle(self, angle: int=0):
        """カメラアングル変更 0x30

        Parameters
        ----------
        angle -90 to 90 [deg] : int
        """
        wdata = int(angle * 127/90) + 128
        self._sendframe(rw=0x0, addr=0x30, wdata=wdata)

if __name__ == "__main__":
    import time
    car = CarDevice()
    while True:
        print(car.get_sensordata())
        time.sleep(0.1)
