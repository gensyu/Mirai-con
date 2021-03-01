import numpy as np
import math

def map_to_anguler_domain(rad, area="all"):
    # (-π,π)にマップ
    if area == "all":
        if rad < 0 or 2*math.pi < rad: # まず(0,2π)にマップ (剰余計算が負の数に対して使いづらいため)
            rad = rad % (2*math.pi)
        if math.pi < rad: # (-π,π)にマップ
            rad = rad - 2*math.pi

    # (-π/2,π/2)にマップ
    if area == "half":
        rad = rad % math.pi # まず(0,π)にマップ
        if math.pi/2 < rad:
            rad = rad - 2*math.pi # (-π/2, π/2)にマップ

    return rad

def Rx(roll):
    cos = math.cos(roll)
    sin = math.sin(roll)
    mat_tmp = np.mat([
        [1, 0, 0],
        [0, cos, -sin],
        [0, sin, cos]
    ])
    return mat_tmp

def Ry(pitch):
    cos = math.cos(pitch)
    sin = math.sin(pitch)
    mat_tmp = np.mat([
        [cos, 0, sin],
        [0, 1, 0],
        [-sin, 0, cos]
    ])
    return mat_tmp

def Rz(yaw):
    cos = math.cos(yaw)
    sin = math.sin(yaw)
    mat_tmp = np.mat([
        [cos, -sin, 0],
        [sin, cos, 0],
        [0, 0, 1]
    ])
    return mat_tmp

def euler_rotate(roll_input, pitch_input, yaw_input):
    tmp1 = Rx( roll_input )
    tmp2 = Ry( pitch_input )
    tmp3 = Rz( yaw_input )
    return tmp1 * tmp2 * tmp3

def accel2roll(vec_acc):
    ay = vec_acc[1,0]
    az = vec_acc[2,0]
    roll = -math.atan(ay / az)

    if ay>0 and az>0 :
        roll = roll + math.pi
    elif ay<0 and az>0:
        roll = roll - math.pi
    
    # (-π,π)にマップ
    roll = map_to_anguler_domain(roll, area = "all")
    
    return roll

def accel2pitch(vec_acc):
    ax = vec_acc[0,0]
    ay = vec_acc[1,0]
    az = vec_acc[2,0]

    # ピッチを計算
    pitch = - math.atan(ax / math.sqrt(ay**2 + az**2))
    # if ax>0 and az>0 :
    #     pitch -= math.pi
    # elif ax<0 and az>0:
    #     pitch += math.pi
    
    # (-π,π)にマップ
    pitch = map_to_anguler_domain(pitch, area = "all")
    
    return pitch

def magnet2yaw(vec_acc, vec_mag):
    # ロールとピッチを補正
    roll = accel2roll(vec_acc)
    pitch = accel2pitch(vec_acc)
    vec_mag = Ry(-pitch) * Rx(-roll) * vec_mag

    mx = vec_mag[0,0]
    my = vec_mag[1,0]

    # ヨーを計算
    yaw = math.atan(my/mx)
    if mx<0 and my>0:
        yaw += math.pi
    elif mx<0 and my<0:
        yaw -= math.pi
    
    # (-π,π)にマップ
    yaw = map_to_anguler_domain(yaw)
    return yaw

def calc_attitude(vec_acc, vec_mag):
    """
    加速度と磁束密度から機体のロール・ピッチ・ヨーを計算する。
    機体に傾きが無く、真北(0,0,0)を向いている状態から、
    x軸回転(ロール) → y軸回転(ピッチ) → z軸回転(ヨー)
    のように機体を回転させた状態を現在の状態としたときの
    ロール・ピッチ・ヨーが返り値。
    座標軸、回転正方向は右手系に従う。

    Parameters
    ----------
    vec_acc : numpy.matrix 3x1
        加速度ベクトル
    vec_mag : numpy.matrix 3x1 
        磁束密度ベクトル

    Returns
    -------
    attitude : numpy.matrix 3x1
        (ロール, ピッチ, ヨー) の3次元ベクトル
    """
    roll = accel2roll(vec_acc)
    pitch = accel2pitch(vec_acc)
    yaw = magnet2yaw(vec_acc, vec_mag)

    attitude = np.mat([roll, pitch, yaw]).T
    return attitude

def testfn():
    axis, point = plot3D.init()

    t = 0
    while True:
        cos = np.cos(t)
        sin = np.sin(t)

        ax = 0
        ay = 0
        az = -1

        mx = 1
        my = 0
        mz = 0

        vec_acc = np.mat([ax,ay,az]).T
        vec_mag = np.mat([mx,my,mz]).T

        Rmat = euler_rotate(0, 0, t)
        vec_acc = Rmat * vec_acc
        vec_mag = Rmat * vec_mag


        attitude = calc_attitude(vec_acc, vec_mag)


        roll = attitude[0]
        pitch = attitude[1]
        yaw = attitude[2]

        basis_x = np.mat([1, 0, 0]).T
        basis_y = np.mat([0, 1, 0]).T
        direction = euler_rotate(-roll, -pitch, -yaw).T * basis_x

        plot3D.plot_vector(direction, point)
        t += 0.05
        plt.pause(.01)
        time.sleep(0.1)

        print(roll, pitch, yaw, map_to_anguler_domain(t))

if __name__ == "__main__":
    import plot3D
    from mpl_toolkits.mplot3d import Axes3D
    from matplotlib.pyplot import axes
    import matplotlib.pyplot as plt
    import sys
    import time

    try:
        testfn()
    except KeyboardInterrupt:
        print("Program ended by user.\n")
        sys.exit()
    except:
        print("不明なエラー")
        raise