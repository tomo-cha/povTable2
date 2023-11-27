# povTable2
2023 Nov 25,26th SFC万学博覧会>ORF

### hardware
- ESP32 Dev Module 
- ブラシレスモーター 
- フォトリフレクタ
- Adafruit DotStar フルカラーLEDテープ
- raspberry pi
- OpenCR
- dynamixelモーター

### software
- src/main.cpp  esp32に書き込む
- src/security.h    wifiのssid, password情報
- src/mking.py         textとcolorから表示する画像を作る
- src/pic_polarConv.py 画像を極座標変換してudp通信で送信し続ける
