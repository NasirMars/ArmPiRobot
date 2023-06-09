import pygame
import rospy
import time
import cv2
import math
import svs
from chassis_control.msg import *

if sys.version_info.major == 2:
    print('このプログラムをPython3で実行してください！')
    sys.exit(0)

print('''
----------------------------------------------------------
----------------------------------------------------------
ヒント：停止するには、Control+Cを押してください。
----------------------------------------------------------
----------------------------------------------------------
''')

start = True
def get_Xbox_Input(prev_values, Init_speed, last_output_time, interval):
    speedX = 0  # X軸方向のスピード
    speedY = 0  # Y軸方向のスピード
    angleX = 0  # X軸方向の角度
    angleY = 0  # Y軸方向の角度

    for event in pygame.event.get():
        if event.type == pygame.JOYAXISMOTION:
            if event.axis in (0, 1):
                prev_values[event.axis] = event.value   # ジョイスティックの値を更新

    current_time = time.time()   # 現在時刻を取得
    if current_time - last_output_time >= interval:
        last_output_time = current_time
        for axis, value in enumerate(prev_values):
            if axis == 0:
                speedX = int(abs(value) * 10) * Init_speed   # X軸方向のスピードを計算
                if int(value * 10) >= 0:
                    angleX = 0   # X軸方向の角度を設定（正方向）
                else:
                    angleX = 180   # X軸方向の角度を設定（逆方向）

            if axis == 1:
                speedY = int(abs(value) * 10) * Init_speed   # Y軸方向のスピードを計算
                if int(value * 10) <= 0:
                    angleY = 90   # Y軸方向の角度を設定（正方向）
                else:
                    angleY = 270   # Y軸方向の角度を設定（逆方向）

    return speedX, angleX, speedY, angleY, last_output_time




def print_Value(speedX, angleX, speedY, angleY):  # 結果を出力
    if angleY !=0:
        if angleY == 90:
            if angleX == 0:
                angle_rad = math.atan2(speedY, speedX)
            else:
                angle_rad = math.atan2(speedY, -speedX)
        else:
            if angleX == 0:
                angle_rad = math.atan2(-speedY, speedX)
            else:
                angle_rad = math.atan2(-speedY, -speedX)
        angle = math.degrees(angle_rad)
        if angle < 0:
            angle += 360

        speed = math.sqrt(pow(speedX, 2)+pow(speedY, 2))
        # print('Angle:'+str(int(angle))+', Speed:', str(int(speed)))
    return (angle, speed)
def stop():
    global start
    start = False
    print("終了中...")
    set_velocity.publish(0, 0, 0)

def send_Command(speed, angle, turn):
    set_velocity.publish(speed, angle, turn)

if __name__ == '__main__':
    Init_speed = 15  # 1-15  スピードの初期値（1から15の範囲）
    turn = 0
    speedX = 0  # X軸方向のスピード
    speedY = 0  # Y軸方向のスピード
    angleX = 0  # X軸方向の角度
    angleY = 0  # Y軸方向の角度

    WHITE_COLOR = (224, 224, 224)    # 白色のRGB値
    BLACK_COLOR = (0, 0, 0)          # 黒色のRGB値
    RED_COLOR = (0, 0, 255)          # 赤色のRGB値
    GREEN_COLOR = (0, 255, 0)        # 緑色のRGB値
    BLUE_COLOR = (255, 0, 0)         # 青色のRGB値
    GOLD_COLOR = (0, 215, 255)       # 金色のRGB値
    CUSTOM_COLOR = (0, 255, 0)       # カスタム色のRGB値

    cap = cv2.VideoCapture(0)   # カメラキャプチャの初期化

    pygame.init()   # Pygameの初期化
    pygame.joystick.init()   # ジョイスティックの初期化
    screen = pygame.display.set_mode((400, 300))   # ウィンドウの設定
    pygame.display.set_caption("スマートカーコントロール")   # ウィンドウのキャプションを設定

    if pygame.joystick.get_count() > 0:
        joystick = pygame.joystick.Joystick(0)
        print("おめでとうございます！Xboxコントローラーの接続が正常に完了しました。")
    else:
        print("申し訳ありませんが、現在Xboxは利用できません。接続を確認してください。")
    # rospy.init_node('car_move_demo', log_level=rospy.DEBUG)
    # rospy.on_shutdown(stop)
    # set_velocity=rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)

    prev_values = [0] * joystick.get_numaxes()   # 前回のジョイスティックの値を保持するリスト
    interval = 0.025   # 出力間隔（秒）
    last_output_time = time.time()   # 前回の出力時刻を保持する変数

    while start:
        success, img = cap.read()   # カメラ画像のキャプチャ
        if not success:
            break

        frame_size = img.shape   # 画像サイズを取得
        max_dimension = max(frame_size[0], frame_size[1])   # 画像の最大次元を取得
        scale = 800 / max_dimension   # スケーリング係数の計算
        img = cv2.resize(img, None, fx=scale, fy=scale)   # 画像のリサイズ
        imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)   # 画像の色空間をBGRからRGBに変換

        speedX, angleX, speedY, angleY, last_output_time=get_Xbox_Input(prev_values, Init_speed, last_output_time, interval)
        angle, speed =print_Value(speedX, angleX, speedY, angleY)
        send_Command(speed, angle, 0)