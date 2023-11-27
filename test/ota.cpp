/*
ota
*/

// 外部ファイル
#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <Adafruit_DotStar.h>

#include "graphics.h"
#include "security.h"

// #define DEBUG

// OTA

// LED
const int VDATAPIN = 23;
const int VCLOCKPIN = 18;
const int HDATAPIN = 13;
const int HCLOCKPIN = 14;
Adafruit_DotStar vstrip(NUMPIXELS, VDATAPIN, VCLOCKPIN, DOTSTAR_BGR); // DOTSTATR_BRGなどでも設定可能 RGB, RBG, BRG,
Adafruit_DotStar hstrip(NUMPIXELS, HDATAPIN, HCLOCKPIN, DOTSTAR_BGR);
int stateRot = 0;
int numDiv = 0;
int numFrame = 0;

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
    Serial.println("Booting");
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    while (WiFi.waitForConnectResult() != WL_CONNECTED)
    {
        Serial.println("Connection Failed! Rebooting...");
        delay(5000);
        ESP.restart();
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

    Serial.println("Ready");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());

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
    int start = micros();

    pov(); // LED, フォトリフレクタ
    rotate();

    int goal = micros();

    Serial.print("loop time = ");
    Serial.println(goal - start);
    Serial.print("rotTime = ");
    Serial.println(rotTime);
}