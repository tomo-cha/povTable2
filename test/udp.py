# -*- coding: utf-8 -*-
# 画像一枚を1つのファイルに変換するコード
import socket
import time
import cv2
import os
import math
import sys
from PIL import Image

# args = sys.argv

# 配列設定
PIXELS = 22  # LED1本あたりのセル数
NUMTAPES = 5  # 繋げるLEDの本数
Div = 60  # 1周の分割数
l = [[0] * PIXELS*NUMTAPES for i in range(Div)]  # RGBを格納するためのリスト宣言・初期化

Bright = 20     # 輝度
Led0Bright = 2  # 中心LEDの輝度 [%]

# ファイル作成
file = open('test/udp.h', 'w')
file.write("send data \n\n")

# 画像ファイルを読み込む(png,jpg,bmpなどが使用可能)
pic1 = "src/bangaku/ban.png"  # 1枚目の画像
pic2 = "src/bangaku/noImage.png"  # 2枚目の画像
pic3 = "src/bangaku/noImage.png"  # 3枚目の画像
pic4 = "src/bangaku/noImage.png"  # 4枚目の画像
pic5 = "src/bangaku/noImage.png"  # 5枚目の画像


# 画像変換関数
def polarConv(pic, n):
    # 画像データ読み込み
    imgOrgin = cv2.imread(pic)

    # 画像サイズ取得
    h, w, _ = imgOrgin.shape

    # 画像縮小
    # 画像のh:画像のw = (PIXELS * 2 -1):?. の比を取って縮小してる
    imgRedu = cv2.resize(imgOrgin, (math.floor(
        (PIXELS * 2 - 1)/h * w), PIXELS * 2 - 1))

    # 縮小画像中心座標
    h2, w2, _ = imgRedu.shape
    wC = math.floor(w2 / 2)
    hC = math.floor(h2 / 2)

    # 極座標変換画像準備
    # 第二引数にサイズ、第三引数にRGBの各色を指定する。
    # imgPolar = Image.new('RGB', (PIXELS, Div))

    # 極座標変換
    for j in range(0, Div):
        # file.write('\t{')
        for i in range(0, hC+1):
            # 座標色取得
            # 参考：http://peaceandhilightandpython.hatenablog.com/entry/2016/01/03/151320
            rP = int(imgRedu[hC - math.ceil(i * math.cos(2*math.pi/Div*j)),
                             wC + math.ceil(i * math.sin(2*math.pi/Div*j)), 2]  # Rを取得
                     * ((100 - Led0Bright) / PIXELS * i + Led0Bright) / 100 * Bright / 100)  # 明るさ調整
            gP = int(imgRedu[hC - math.ceil(i * math.cos(2*math.pi/Div*j)),
                             wC + math.ceil(i * math.sin(2*math.pi/Div*j)), 1]  # Gを取得
                     * ((100 - Led0Bright) / PIXELS * i + Led0Bright) / 100 * Bright / 100)  # 明るさ調整
            bP = int(imgRedu[hC - math.ceil(i * math.cos(2*math.pi/Div*j)),
                             wC + math.ceil(i * math.sin(2*math.pi/Div*j)), 0]  # Bを取得
                     * ((100 - Led0Bright) / PIXELS * i + Led0Bright) / 100 * Bright / 100)  # 明るさ調整
    
            # udp設定
            if (n % 2 == 1):
                l[j][i+(n-1)*PIXELS] = '%02X%02X%02X' % (rP, gP, bP)
            else:
                l[j][abs((PIXELS*n-1)-i)] = '%02X%02X%02X' % (rP, gP, bP)


# 変換, lに格納
polarConv(pic1, 1)  # 0~23番目のセル l[j][i]
polarConv(pic2, 2)  # 47~24番目のセル l[j][47-i]
polarConv(pic3, 3)  # 48~71番目のセル l[j][i+48]
polarConv(pic4, 4)  # 95~72番目のセル l[j][95-i]
polarConv(pic5, 5)  # 96~119番目のセル l[j][i+96]


# udp設定
sendAddr = ('192.168.179.3', 1234)  # ポート番号は1234
udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

for k in range(2): #パケットロスがあるので3回送る
    for j in range(0, Div):
        data = '%02X' % j
        for i in range(0, PIXELS*NUMTAPES):
            data+=l[j][i]
            if i == PIXELS*NUMTAPES-1:
                udp.sendto(data.encode('utf-8'), sendAddr)
                time.sleep(0.001) #sleepがないとパケットロスが激増する
                file.write(data+"\n")
                print(data.encode('utf-8'))

udp.close()
file.close()