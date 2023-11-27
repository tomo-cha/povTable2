/*
udpとtcp + otaの合体
pythonはそれぞれの.pyを実行する
*/

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <AsyncUDP.h>
#include <ArduinoOTA.h>
#include <Adafruit_DotStar.h>

#include "security.h"

// tcp
const int serverPort = 5000; // サーバーポート番号
WiFiServer server(serverPort);
WiFiClient client;

// udpで画像受信
AsyncUDP udp;

// LED
const int NUMPIXELS = 22 * 5;
const int Div = 60;

uint32_t vpic[Div][NUMPIXELS] = {
    0,
};
char chararrayDiv[] = "0x00";
char chararrayColor[] = "0xffffff";
int stateRot = 0;
int numDiv = 0;
const int VDATAPIN = 23;
const int VCLOCKPIN = 18;
const int HDATAPIN = 13;
const int HCLOCKPIN = 14;
Adafruit_DotStar vstrip(NUMPIXELS, VDATAPIN, VCLOCKPIN, DOTSTAR_BGR); // DOTSTATR_BRGなどでも設定可能 RGB, RBG, BRG,
Adafruit_DotStar hstrip(NUMPIXELS, HDATAPIN, HCLOCKPIN, DOTSTAR_BGR);

// モータードライバ
const int motorPin = 26; // A19 速度指令
const int turnPin = 27;  // 回転方向

// フォトリフレクタ
const int PHOTOPIN = 33;
const int th_PHOTOSENSORS = 2000; // 赤外線センサの閾値
unsigned long rotTime, timeOld, timeNow;
float rpm;

void rotate()
{
    /*
    回転方向と回転速度を指定
    回転を始める...だんだん早くする。約3秒かけてmax速度に
    回転中
    回転を終わらせる...だんだん遅くする
    回転停止...停止のアルゴリズム
    */
    digitalWrite(turnPin, LOW);
    ledcWrite(0, 120); // 120が適正
}

void draw_status()
{
    // 何番目のLEDを何色に光らせるのかをset
    vstrip.clear();
    hstrip.clear(); // 一つ前の点灯パターンを消さないとそのまま残る。これがないと画像が回転しているように見える

    for (int i = 0; i < NUMPIXELS; i++)
    {
        vstrip.setPixelColor(i, vpic[(numDiv + (Div * 3 / 4)) % Div][i]); // フォトリフレクタの位置が270度ずれたため、開始位置も3/4ずれる
        hstrip.setPixelColor(i, vpic[(numDiv + (Div / 4)) % Div][i]);     // 半分ずれる
    }

    // setした通りにLEDを光らせる
    vstrip.show();
    hstrip.show();

    // 経過時間から何番目のパターンを光らせるか決める。
    if (micros() - timeOld > rotTime / Div * (numDiv + 1))
    {
        numDiv++;
        if (numDiv >= Div - 1)
        {
            numDiv = 0;
        }
    }
}

void pov()
{
    // フォロリフレクタの反応時の処理(排他処理をしている)
    if (stateRot == 0 && analogRead(PHOTOPIN) < th_PHOTOSENSORS) // 閾値以下=反応
    {
        timeNow = micros();
        rotTime = timeNow - timeOld;
        timeOld = timeNow;
        rpm = 60000000 / rotTime;
        // numDiv = 0; // フォトリフレクタの位置を描画開始場所にする
        stateRot = 1;
    }
    if (stateRot == 1 && analogRead(PHOTOPIN) > th_PHOTOSENSORS)
    {
        stateRot = 0;
    }

    draw_status(); // 読み込んだ画像通りにLEDを光らせる
}

void setup()
{
    // Serial通信
    Serial.begin(115200);

    // wifi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    if (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        Serial.println("WiFi Failed");
        while (1)
        {
            delay(1000);
        }
    }

    // ArduinoOTA
    ArduinoOTA.setPort(3232);           // Port defaults to 3232
    ArduinoOTA.setHostname("povTable"); // Hostname defaults to esp3232-[MAC]
    ArduinoOTA.setPassword("admin");    // No authentication by default
    // // Password can be set with it's md5 value as well
    // // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
    // // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
    ArduinoOTA.onError([](ota_error_t error)
                       {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

    ArduinoOTA.begin();

    // tcp
    server.begin();
    Serial.println("Server started");
    // UDP受信
    if (udp.listen(1234)) // python側とポートを合わせる。自由な数字で良い
    {
        Serial.print("UDP Listening on IP: ");
        Serial.println(WiFi.localIP());
        udp.onPacket([](AsyncUDPPacket packet)
                     {
                         chararrayDiv[2] = packet.data()[0];
                         chararrayDiv[3] = packet.data()[1];
                         Serial.print("strtoul=");
                         Serial.println(int(strtoul(chararrayDiv, NULL, 16))); // パケットロスをしらべる
                         // for (int k = 0; k < Frame; k++) { //gif用
                         for (int i = 0; i < NUMPIXELS; i++)
                         {
                             for (int j = 0; j < 6; j++)
                             {
                                 chararrayColor[j + 2] = packet.data()[2 + i * 6 + j];
                             }
                             vpic[int(strtoul(chararrayDiv, NULL, 16))][i] = strtoul(chararrayColor, NULL, 16);
                         }
                         //      }
                     });
    }

    // モーター
    // 使用するタイマーのチャネルと周波数を設定
    ledcSetup(0, 15000, 8); // 30kHz
    // motorPinをチャネル0へ接続
    ledcAttachPin(motorPin, 0);
    pinMode(turnPin, OUTPUT);
    digitalWrite(turnPin, LOW); // LOWが正回転

    // LED tape
    vstrip.begin();
    hstrip.begin();
    vstrip.clear();
    hstrip.clear();
    vstrip.show();
    hstrip.show();
}

void loop()
{
    ArduinoOTA.handle();
    // tcp
    client = server.available();

    if (client)
    {
        Serial.println("New client connected");

        // ESP32から"0"を受信
        if (client.available())
        {
            char c = client.read();
            if (String(c) == "0")
            {
                Serial.println("Received: " + String(c));
                // ラズベリーパイに"a"を送信
                client.print("a");
            }
            else if (String(c) == "1")
            {
                Serial.println("Received: " + String(c));
                // ラズベリーパイに"b"を送信
                client.print("b");
            }
            else if (String(c) == "2")
            {
                Serial.println("Received: " + String(c));
                // ラズベリーパイに"b"を送信
                client.print("c");
            }
        }

        // クライアントが切断されたときの処理
        if (!client.connected())
        {
            Serial.println("Client disconnected");
            client.stop();
        }
    }
    int start = micros();

    pov();    // LED, フォトリフレクタ
    rotate(); // 回転

    int goal = micros();

    // Serial.print("loop time = ");
    // Serial.println(goal - start);
    // Serial.print("rotTime = ");
    // Serial.println(rotTime);
    // Serial.println("vpic = ");
    // for (int j = 0; j < Div; j++)
    // {
    //     for (int i = 0; i < NUMPIXELS; i++)
    //     {
    //         Serial.print(vpic[j][i]);
    //     }
    //     Serial.print("\n");
    // }
}