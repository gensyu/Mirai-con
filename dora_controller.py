# 自己位置推定と機体制御のプログラム
# (Self Position Estimation and Control)

import numpy as np
import time
import math
import debug_comm
import calc_attitude

class car_controller:
    def __init__(self):
        self.car = debug_comm.CarDevice()
        self.state = {
            "stage": 1,
            "start_time": 0,
            "elapsed_time": 0,
            "emergency": False,
        }
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

        # ■ walldst (n, e, w, s) [mm]:
        # 各ステージのXY方向の壁までの距離。
        # Startを右上、Goalを左下になるようにコースを俯瞰したときの上方向を北として
        # 北：n, 東：e, 西：w, 南：s
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
            "n": 100.0,
            "e": 100.0,
            "w": 100.0,
            "s": 100.0,
        }
        
        self.att = {
            "roll": 0.0, # [rad]
            "pitch": 0.0, # [rad]
            "yaw": 0.0, # [rad]
        }
        
        # 一時保存用。i成分を読み出すときは、att_tmp = [i,0]とする。
        self.att_tmp = np.mat([0.0, 0.0, 0.0]).T 

        # 地磁気センサのキャリブレーションに利用する蓄積用変数
        self.magx_accum = np.array([])
        self.magy_accum = np.array([])
        self.magz_accum = np.array([])

        self.magx_offset = (2629 - 2024) / 2 - 95.5
        self.magy_offset = (1871 - 2582) / 2 - 92.5
        self.magz_offset = (2448 - 1834) / 2  + 344.5

        # オイラー角のオフセット
        self.roll_offset = 0 # 確定 # [rad]
        self.pitch_offset = 3.14 # 確定 # [rad]
        # self.yaw_offset = 1.47 # 暫定（安藤さん宅の壁に平行にした設定） # [rad]
        self.yaw_offset = 1.47 + np.deg2rad(-22) # 暫定（自宅の壁に平行にした設定） # [rad]

        # 東西南北のマスター値。現地で調整 [deg]
        self.yaw_north = -100
        self.yaw_east = 180
        self.yaw_west = 0
        self.yaw_south = 75
        

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
        self.att["yaw"] = calc_attitude.map_to_anguler_domain(self.att_tmp[2,0] - self.yaw_offset) # / math.pi * 180
        
        
        # TOFセンサが壁に垂直な状態から ±ang [rad]の時のみ壁までの距離を算出する。
        ang = 30
        yaw_rad = self.att["yaw"]
        yaw_deg = np.rad2deg(yaw_rad)
        if self.yaw_west - ang < yaw_deg and yaw_deg < self.yaw_west + ang: # 西向きの時
            self.dst["n"] = math.cos(yaw_rad) * self.sensor["tof_r"]
            self.dst["e"] = math.cos(yaw_rad) * self.sensor["tof_b"]
            self.dst["s"] = math.cos(yaw_rad) * self.sensor["tof_l"]
            self.dst["w"] = math.cos(yaw_rad) * self.sensor["tof_f"]
        elif self.yaw_south - ang < yaw_deg and yaw_deg < self.yaw_south + ang: # 南向きの時
            yaw_rad = yaw_rad - np.deg2rad(self.yaw_south)
            self.dst["n"] = math.cos(yaw_rad) * self.sensor["tof_b"]
            self.dst["e"] = math.cos(yaw_rad) * self.sensor["tof_l"]
            self.dst["s"] = math.cos(yaw_rad) * self.sensor["tof_f"]
            self.dst["w"] = math.cos(yaw_rad) * self.sensor["tof_r"]
        elif - ang < abs(yaw_deg) - self.yaw_east and abs(yaw_deg) - self.yaw_east < + ang : # 東向きの時
            yaw_rad = abs(yaw_rad) - np.deg2rad(self.yaw_east)
            self.dst["n"] = math.cos(yaw_rad) * self.sensor["tof_l"]
            self.dst["e"] = math.cos(yaw_rad) * self.sensor["tof_f"]
            self.dst["s"] = math.cos(yaw_rad) * self.sensor["tof_r"]
            self.dst["w"] = math.cos(yaw_rad) * self.sensor["tof_b"]
        elif self.yaw_north - ang < yaw_deg and yaw_deg < self.yaw_north + ang: # 北向きの時
            yaw_rad = yaw_rad - np.deg2rad(self.yaw_north)
            self.dst["n"] = math.cos(yaw_rad) * self.sensor["tof_f"]
            self.dst["e"] = math.cos(yaw_rad) * self.sensor["tof_r"]
            self.dst["s"] = math.cos(yaw_rad) * self.sensor["tof_b"]
            self.dst["w"] = math.cos(yaw_rad) * self.sensor["tof_l"]
        else:
            self.dst["n"] = -1
            self.dst["e"] = -1
            self.dst["s"] = -1
            self.dst["w"] = -1

    def debug_state(self):
        '''
        機体の姿勢と壁までの距離をコンソールに表示する。デバッグ用。
        '''
        print("roll:{0:+06.1f}, pitch:{1:+06.1f}, yaw:{2:+06.1f}, dst[n]:{3:+07.1f}, dst[e]:{4:+07.1f}, dst[w]:{5:+07.1f}, dst[s]:{6:+07.1f}, tof_f:{7:+07.1f}, tof_r:{8:+07.1f}, tof_l:{9:+07.1f}, tof_b:{10:+07.1f}" \
            .format(
                np.rad2deg(self.att["roll"]), np.rad2deg(self.att["pitch"]), np.rad2deg(self.att["yaw"]),
                self.dst["n"], self.dst["e"], self.dst["w"], self.dst["s"],
                self.sensor["tof_f"], self.sensor["tof_r"], self.sensor["tof_l"], self.sensor["tof_b"],
            )
        )
    
    def calibration(self, n_data = 3000):
        '''
        地磁気センサ校正用。各軸のmin-maxの中間の値をオフセットとする。
        '''
        for i in range(n_data):
            self.get_sensordata()
            self.magx_accum = np.append(self.magx_accum, self.sensor["mag_x"])
            self.magy_accum = np.append(self.magy_accum, self.sensor["mag_y"])
            self.magz_accum = np.append(self.magz_accum, self.sensor["mag_z"])
            print(i, ": ", self.sensor["mag_x"], " ", self.sensor["mag_y"], " ", self.sensor["mag_z"])
            time.sleep(0.02)

        x_max = max(self.magx_accum)
        x_min = min(self.magx_accum)
        y_max = max(self.magy_accum)
        y_min = min(self.magy_accum)
        z_max = max(self.magz_accum)
        z_min = min(self.magz_accum)

        # self.magx_offset = (x_max + x_min)/2
        # self.magy_offset = (y_max + y_min)/2
        # self.magz_offset = (z_max + z_min)/2

        print("プログラム中のself.magX_offset, (X=x,y,z) の値を以下の値に書き換えてください")
        print("x_offset: ", (x_max + x_min)/2)
        print("y_offset: ", (y_max + y_min)/2)
        print("z_offset: ", (z_max + z_min)/2)

    def r_motor(self, duty):
        '''
        0 < duty < 1 : 正転
        -1 < duty < 0 : 逆転
        '''
        if duty >= 0:
            self.car.right_motor(debug_comm.MOTORDIR.CW, duty)
        elif duty < 0:
            self.car.right_motor(debug_comm.MOTORDIR.CCW, -duty)

    def l_motor(self, duty):
        '''
        0 < duty < 1 : 正転
        -1 < duty < 0 : 逆転
        '''
        if duty >= 0:
            self.car.left_motor(debug_comm.MOTORDIR.CW, duty)
        elif duty < 0:
            self.car.left_motor(debug_comm.MOTORDIR.CCW, -duty)
    
    def stop_motor(self):
        self.r_motor(0)
        self.l_motor(0)

    def update(self):
        '''
        状態更新
        '''
        self.get_sensordata()
        self.cal_state()
        self.state["elapsed_time"] = time.time() - self.state["start_time"]
        self.debug_state()
        time.sleep(0.012)

    def turn_to(self, yaw_target, max_duty = 1):
        '''
        目標のヨー角を向くまでその場で旋回
        Args: yaw_target [deg]
        '''
        ang_max = math.pi/4 # 45°
        yaw_target = np.deg2rad(yaw_target)
        yaw_measured = self.att["yaw"]
        diff = yaw_measured - yaw_target
        diff = calc_attitude.map_to_anguler_domain(diff)
        
        while abs(diff) > np.deg2rad(5):
            print(diff)
            self.update()
            yaw_measured = self.att["yaw"]
            diff = yaw_measured - yaw_target
            diff = calc_attitude.map_to_anguler_domain(diff)
            # ang_max以上はang_maxとして扱い、diffをang_maxで規格化
            if abs(diff) > ang_max:
                diff = np.sign(diff) * ang_max
            diff = diff / ang_max

            if diff >= 0:
                # 右旋回
                self.r_motor(0)
                self.l_motor((diff+1)/2 * max_duty)
            elif diff < 0:
                # 左旋回
                self.r_motor((-diff+1)/2 * max_duty)
                self.l_motor(0)
    
    def move_with_yaw_ctrl(self, yaw_target, max_duty = 1):
        '''
        目標のヨー角を保つように前方に移動
        input: yaw_target [deg]
        '''
        ang_max = math.pi/4 # 45°

        # radに変換
        yaw_target = np.deg2rad(yaw_target)
        yaw_measured = self.att["yaw"]

        diff = yaw_measured - yaw_target
        diff = calc_attitude.map_to_anguler_domain(diff)
        
        # ang_max以上はang_maxとして扱い、diffをang_maxで規格化
        if abs(diff) > ang_max:
            diff = np.sign(diff) * ang_max
        diff = diff / ang_max

        if diff >= 0:
            # 右旋回
            self.r_motor((1-diff) * max_duty)
            self.l_motor(1 * max_duty)
        elif diff < 0:
            # 左旋回
            self.r_motor(1 * max_duty)
            self.l_motor((1+diff) * max_duty)

    def move_along_wall(self, target_wall, yaw_target, walldist_target, wall_orientation = "r", max_duty = 1):
        '''
        壁沿いに移動する。
        Args:
            target_wall (str): どの壁に対する距離を制御するか。"n", "e", "w", "s"
            yaw_target (float): 進行方向のyaw
            wall_orientation (str): 制御対象の壁が左右どちらにあるか。"r" or "l"
            walldist_target (float): 壁までの距離の目標値 [mm]
            max_duty (float): 最大のduty比　0.0～1.0
        '''
        
        yaw_measured = self.att["yaw"]
        walldist_measured = self.dst[target_wall]

        diff_yaw = yaw_measured - yaw_target
        diff_yaw = calc_attitude.map_to_anguler_domain(diff_yaw)
        diff_dist = walldist_measured - walldist_target
        
        # 角度がつきすぎている or 壁までの距離がエラーの場合、角度制御に切り替える
        if abs(diff_yaw) > 30 or walldist_measured == -1:
            self.move_with_yaw_ctrl(yaw_target, max_duty)
            return 0

        dist_max = 100 # [mm]
        if abs(diff_dist) >= dist_max:
            diff_dist = np.sign(diff_dist)
        elif abs(diff_dist) < dist_max:
            diff_dist = diff_dist / dist_max
        
        # 壁までの距離に応じて進行方向を制御
        if wall_orientation == "r":
            self.move_with_yaw_ctrl(yaw_target - 15 * diff_dist, max_duty)
        elif wall_orientation == "l":
            self.move_with_yaw_ctrl(yaw_target + 15 * diff_dist, max_duty)
        print(diff_dist)
    
    def move_to_limit(self, heading, guide_wall = "n", wall_orientation = "r", move_limit = 150, guide_dist = 100, time_limit = 100):
        # 突き当たるまで進む
        # heading = self.yaw_west
        self.state["start_time"] = time.time()
        print("yes")
        while self.dst["w"] == -1 or self.dst["w"] > move_limit:
            print("yes")
            self.update()
            if self.state["elapsed_time"] > time_limit:
                break
            self.move_along_wall(guide_wall, heading, guide_dist, wall_orientation, 1)

    def stage1(self):
        # 突き当たるまで進む
        self.move_to_limit(
            heading = self.yaw_west,
            guide_wall = "n",
            move_limit = 350,
            guide_dist = 100)
        
        # 南に旋回する
        self.turn_to(self.yaw_south)

        # 突き当たるまで進む
        self.move_to_limit(
            heading = self.yaw_south,
            guide_wall = "w",
            move_limit = 150,
            guide_dist = 200)
        
        # 東に旋回する
        self.turn_to(self.yaw_east)

        # ステージクリア
        self.state["stage"] += 1
    
    def stage2(self):
        # 突き当たるまで進む
        self.move_to_limit(
            heading = self.yaw_east,
            guide_wall = "n",
            wall_orientation = "l",
            move_limit = 150,
            guide_dist = 100,
            time_limit = 5,
        )
        
        # ステージクリア
        self.state["stage"] += 1
    
    def stage3(self):
        pass

    
    def emergency(self):
        pass
    
    def action(self):
        '''
        状態変数をもとに行動を選択する
        '''

        if self.state["emergency"] == True:
            self.emergency()

        if self.state["stage"] == 1:
            self.stage1()
        elif self.state["stage"] == 2:
            self.stage2()
        elif self.state["stage"] == 3:
            self.stage3()
        elif self.state["stage"] == 4:
            self.stage4()
        elif self.state["stage"] == 5:
            self.stage5()
        else:
            self.state["emergency"] = True


if __name__ == "__main__":
    dora = car_controller() # ドラえもん
    dora.connect()
    dora.car.vsc3_disable() # リモコン無効
    # dora.car.vsc3_enable() # リモコン有効
    dora.stop_motor() # モーター停止
    # dora.calibration() # 地磁気センサ校正用関数
    dora.update()

    while True:
        try:
            # コース攻略
            dora.action()

            # 壁に沿って移動
            # dora.update()
            # dora.move_along_wall("n", dora.yaw_west, 100, wall_orientation = "r", max_duty = 1)

            # 方角指定で移動
            # dora.update()
            # dora.move_with_yaw_ctrl(dora.yaw_west, max_duty = 0.5)
            
        except KeyboardInterrupt:
            print("Program ended by user")
            dora.stop_motor()
            break
        
        except:
            raise