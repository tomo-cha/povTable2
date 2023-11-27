'''
esp32の代わり
送信のための準備
送信先のip
送信先のport
socket作成
connectする
sendする
応答を受信
socketを閉じる

受信のための準備
受信先のip
受信先のport
socket作成
bind
サーバーの起動listen
接続するaccept
データを受信recv
socketをクローズ

動け
ロック1, 2をはずす
send 0

止まれ
send 1
recv a
ロック1する
send 2
recv b
ロック2する
send 3
'''
import socket

# 受信
recv_ip = "127.0.0.1"
recv_port = 5002

# 送信
send_ip = "127.0.0.1"
send_port = 5001


# ソケットを作成
recv_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# バインド
recv_socket.bind((recv_ip, recv_port))

# ソケットを作成
send_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)




# 受信ループ
while True:
    # データを受信
    data, client_address = recv_socket.recvfrom(1024)

    # 受信したデータを表示
    print(f"Received data from {client_address}: {data.decode()}")

    if(data.decode() == "0"):
        # 送信するデータ
        message = "0_OK"
        # データを送信
        print(message)
    elif(data.decode() == "1"):
        # 送信するデータ
        message = "a"
        # データを送信
        send_socket.sendto(message.encode(), (send_ip, send_port))
    elif(data.decode() == "2"):
        # 送信するデータ
        message = "b"
        # データを送信
        send_socket.sendto(message.encode(), (send_ip, send_port))
    elif(data.decode() == "3"):
        # 送信するデータ
        message = "3_OK"
        # データを送信
        print(message)
        break


# ソケットを閉じる
recv_socket.close()
send_socket.close()

