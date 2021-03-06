# 自己位置推定と機体制御のプログラム
# (Self Position Estimation and Control)

import numpy as np
import time
import math
import debug_comm
import calc_attitude

class car_controller:
    def __init__(self):
        self.controll_mode = "initialize"
        self.car = debug_comm.CarDevice()
        self.stage = 1
        self.sensor = {
            "tof_f": 100.0,
            "tof_r": 100.0,
            "tof_l": 100.0,
            "tof_b": 100.0,
            "mag_x": 0.0,
            "mag_y": 0.0,
            "mag_z": 0.0,
            "acc_x": 0.0,
            "acc_y": 0.0,
            "acc_z": 0.0,
        }

        # ■ walldst (u,r,l,d) [mm]:
        # 各ステージのXY方向の壁までの距離。
        # Startを右上、Goalを左下になるようにコースを俯瞰したとき
        # 上方向：u, 右方向：r, 左方向：l, 下方向：d
        #  
        # ■ att (roll, pitch, yaw) [(-π, π)^3]:
        # 機体の姿勢を表すオイラー角。(attitude="姿勢")
        # Startを右上、Goalを左下になるようにコースを俯瞰したときの左方向をx軸正方向、
        # 下方向をy軸正方向、鉛直の逆方向をz軸正方向とする。（右手系）
        # 機体に傾きが無く、x軸正方向を向いている状態から、
        # x軸回転(ロール) → y軸回転(ピッチ) → z軸回転(ヨー)
        # のように機体を回転させた状態を現在の状態としたときのroll, pitch, yawをここでは考える。
        # 回転正方向も右手系に従う。
        self.dst = {
            "u": 100.0,
            "r": 100.0,
            "l": 100.0,
            "d": 100.0,
        }
        
        self.att = {
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }
        
        # 一時保存用。i成分を読み出すときは、att_tmp = [i,0]とする。
        self.att_tmp = np.mat([0.0, 0.0, 0.0]).T 

        # 地磁気センサのキャリブレーションに利用する蓄積用変数
        self.magx_accum = np.array([])
        self.magy_accum = np.array([])
        self.magz_accum = np.array([])

        self.magx_offset = (2629 - 2024) / 2
        self.magy_offset = (1871 - 2582) / 2
        self.magz_offset = (2448 - 1834) / 2

        self.roll_offset = 0 # 確定
        self.pitch_offset = 3.14 # 確定
        self.yaw_offset = 1.47 # 暫定（安藤さん宅の壁に平行にした設定）
        

    def connect(self):
        pass

    def get_sensordata(self):
        '''
        センサ情報を取得
        '''
        temp = self.car.get_sensordata()
        self.sensor["tof_f"] = temp["tof_f"]
        self.sensor["tof_r"] = temp["tof_r"]
        self.sensor["tof_l"] = temp["tof_l"]
        self.sensor["tof_b"] = temp["tof_b"]
        self.sensor["mag_x"] = temp["mag_x"] - self.magx_offset
        self.sensor["mag_y"] = temp["mag_y"] - self.magy_offset
        self.sensor["mag_z"] = temp["mag_z"] - self.magz_offset
        self.sensor["acc_x"] = temp["acc_x"]
        self.sensor["acc_y"] = temp["acc_y"]
        self.sensor["acc_z"] = temp["acc_z"]
        
        # print(self.sensor["tof_f"])
        # print(self.sensor["mag_x"])
        # print(self.sensor.values())

    def cal_state(self):
        '''
        self.sensorからself.dst, self.attを計算する
        '''
        
        vec_mag = np.mat( [self.sensor["mag_x"], self.sensor["mag_y"], self.sensor["mag_z"]] ).T
        vec_acc = np.mat( [self.sensor["acc_x"], self.sensor["acc_y"], self.sensor["acc_z"]] ).T
        
        self.att_tmp = calc_attitude.calc_attitude(vec_acc, vec_mag)
        self.att["roll"] = self.att_tmp[1,0] - self.roll_offset # / math.pi * 180 # 機体の設置のロールピッチが逆
        self.att["pitch"] = self.att_tmp[0,0] - self.pitch_offset # / math.pi * 180 # 機体の設置のロールピッチが逆
        self.att["yaw"] = self.att_tmp[2,0] - self.yaw_offset # / math.pi * 180
        
        
        # TOFセンサが壁に垂直な状態から ±ang [rad]の時のみ壁までの距離を算出する。
        ang = 30
        yaw_rad = self.att["yaw"]
        yaw_deg = np.rad2deg(yaw_rad)
        if 0 - ang < yaw_deg and yaw_deg < 0 + ang: # 左向きの時
            self.dst["u"] = math.cos(yaw_rad) * self.sensor["tof_r"]
            self.dst["r"] = math.cos(yaw_rad) * self.sensor["tof_b"]
            self.dst["d"] = math.cos(yaw_rad) * self.sensor["tof_l"]
            self.dst["l"] = math.cos(yaw_rad) * self.sensor["tof_f"]
        elif 90 - ang < yaw_deg and yaw_deg < 90 + ang: # 下向きの時
            yaw_rad = yaw_rad - np.deg2rad(90)
            self.dst["u"] = math.cos(yaw_rad) * self.sensor["tof_b"]
            self.dst["r"] = math.cos(yaw_rad) * self.sensor["tof_l"]
            self.dst["d"] = math.cos(yaw_rad) * self.sensor["tof_f"]
            self.dst["l"] = math.cos(yaw_rad) * self.sensor["tof_r"]
        elif - ang < abs(yaw_deg) - 180 and abs(yaw_deg) - 180 < + ang : # 右向きの時
            yaw_rad = abs(yaw_rad) - np.deg2rad(180)
            self.dst["u"] = math.cos(yaw_rad) * self.sensor["tof_l"]
            self.dst["r"] = math.cos(yaw_rad) * self.sensor["tof_f"]
            self.dst["d"] = math.cos(yaw_rad) * self.sensor["tof_r"]
            self.dst["l"] = math.cos(yaw_rad) * self.sensor["tof_b"]
        elif -90 - ang < yaw_deg and yaw_deg < -90 + ang: # 上向きの時
            yaw_rad = yaw_rad - np.deg2rad(-90)
            self.dst["u"] = math.cos(yaw_rad) * self.sensor["tof_f"]
            self.dst["r"] = math.cos(yaw_rad) * self.sensor["tof_r"]
            self.dst["d"] = math.cos(yaw_rad) * self.sensor["tof_b"]
            self.dst["l"] = math.cos(yaw_rad) * self.sensor["tof_l"]
        else:
            self.dst["u"] = -1
            self.dst["r"] = -1
            self.dst["d"] = -1
            self.dst["l"] = -1

    def deg(self, rad):
        return rad/math.pi*180

    
    def debug_state(self):
        '''
        機体の姿勢と壁までの距離をコンソールに表示する。デバッグ用。
        '''
        print("roll:{0:.2f}, pitch:{1:.2f}, yaw:{2:.2f}, dst[u]:{3:.2f}, dst[r]:{4:.2f}, dst[l]:{5:.2f}, dst[d]:{6:.2f}, " \
            .format(
                np.rad2deg(self.att["roll"]), np.rad2deg(self.att["pitch"]), np.rad2deg(self.att["yaw"]),
                self.dst["u"], self.dst["r"], self.dst["l"], self.dst["d"],
            )
        )

    def move_(self):
        


    
    def dec_strategy(self):
        '''
        行動方針を決定する
        '''
        pass
    
    def action(self):
        '''
        行動する
        '''
        pass

    
    def calibration(self, n_data = 2000):
        
        for i in range(n_data):
            self.get_sensordata()
            self.magx_accum = np.append(self.magx_accum, self.sensor["mag_x"])
            self.magy_accum = np.append(self.magy_accum, self.sensor["mag_y"])
            self.magz_accum = np.append(self.magz_accum, self.sensor["mag_z"])
            print(self.sensor["mag_x"], " ", self.sensor["mag_y"], " ", self.sensor["mag_z"])

        x_max = max(self.magx_accum)
        x_min = min(self.magx_accum)
        y_max = max(self.magy_accum)
        y_min = min(self.magy_accum)
        z_max = max(self.magz_accum)
        z_min = min(self.magz_accum)

        self.magx_offset = (x_max + x_min)/2
        self.magy_offset = (y_max + y_min)/2
        self.magz_offset = (z_max + z_min)/2

        print("end")

    def r_motor(self, duty):
        self.car.right_motor(debug_comm.MOTORDIR.CW, duty)

    def l_motor(self, duty):
        self.car.left_motor(debug_comm.MOTORDIR.CW, duty)


if __name__ == "__main__":
    dora = car_controller() # ドラえもん
    dora.connect()
    dora.car.vsc3_disable() # リモコン無効
    # dora.car.vsc3_enable() # リモコン有効

    # dora.calibration()

    # dora.r_motor(1)
    # dora.l_motor(1)

    # time.sleep(1)
    # dora.r_motor(0)
    # dora.l_motor(0)

    while True:
        try:
            dora.get_sensordata()
            dora.cal_state()
            dora.debug_state()
            # dora.dec_strategy()
            # dora.action()
            time.sleep(0.1)
            
        except KeyboardInterrupt:
            print("Program ended by user")
            break
        
        except:
            raise