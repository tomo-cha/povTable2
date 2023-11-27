#include <WiFi.h>
#include <WiFiClient.h>
#include "security.h"

const int serverPort = 5000; // サーバーポート番号

WiFiServer server(serverPort);
WiFiClient client;

void setup()
{
    Serial.begin(115200);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }

    Serial.println("Connected to WiFi");
    Serial.println("IP address: ");
    Serial.println(WiFi.localIP());

    server.begin();
    Serial.println("Server started");
}

void loop()
{
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
}
