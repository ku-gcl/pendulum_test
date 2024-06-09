# Overview

倒立振子用のテストコード

# 最新の情報

## コンパイルコマンド

コードを編集したらコードをコンパイルする。

```bash
g++ -o /home/ubuntu/pendulum_pj/pendulum_test/bin/PENDULUM /home/ubuntu/pendulum_pj/pendulum_test/main.cpp -lpigpiod_if2 -lrt -pthread

g++ -o /home/ubuntu/pendulum_pj/pendulum_test/bin/PENDULUM_CLEANUP /home/ubuntu/pendulum_pj/pendulum_test/cleanup.cpp -lpigpiod_if2 -lrt
```

# 開発中に使用した内容（Archive）

## スクリプトの実行

```
g++ -o LED LED.cpp -lpigpiod_if2 -lrt
sudo ./LED
```

```
g++ -o driver_test driver_test.cpp -lpigpiod_if2 -lrt
sudo ./driver_test
```

## Kalman filter なしのプログラムを実行

```bash
g++ -o pendulum inverted_pendulum_without_kalman.cpp -lpigpiod_if2 -lrt -pthread
sudo ./pendulum

source ./cleanup.sh
```

## ボタン動作の内容を記述したファイルを実行

```bash
python3 /home/ubuntu/pendulum_project/pendulum_test/pendulum_auto_start_script.py
```

`python`ではなく、`python3`を使用すること。
`python`を使うと、GPIO ピンの ON/OFF が動作しなくなる。

## 電源ライトを点滅させる

```bash
sudo nano /boot/firmware/config.txt
```

config.txt を編集

```config.txt

...

# turn power LED into heartbeat
dtparam=pwr_led_trigger=heartbeat

```

再起動

```bash
sudo reboot
```

## ctrl+C や ctrl+Z のコマンド操作を検知

[C で強制終了を回避(出来るだけ)](https://qiita.com/Ki4mTaria/items/838f81d3eecd5cc7d91e)

## コマンド履歴

```bash
history
```

## C++コードのプロセスを終了

```bash
# 実行
sudo /home/ubuntu/PENDULUM

# 終了
ps aux | grep PENDULUM
sudo killall -9 PENDULUM
sudo /home/ubuntu/PENDULUM_CLEANUP
```

## 自動起動

[systemd を使ってスクリプト自動起動](https://monomonotech.jp/kurage/raspberrypi/systemd_autostart.html)

```bash
sudo touch /etc/systemd/system/pendulum.service
sudo nano /etc/systemd/system/pendulum.service
```

pendulum.service を編集。

```pendulum.service
[Unit]
Description=Execution pendulum auto start script
After=network.target

[Service]
ExecStart=/usr/bin/python3 pendulum_auto_start_script.py
WorkingDirectory=/home/ubuntu/pendulum_project/pendulum_test

[Install]
WantedBy=multi-user.target
```

サービスを有効化するには、以下のコマンドを実行

```bash
# 有効化
sudo systemctl enable pendulum.service

# 無効化
sudo systemctl disable pendulum.service
```

サービスファイルを編集したときは

```bash
sudo systemctl daemon-reload
```

## pigpiod.service を編集

[pigpio Daemon の Option](https://abyz.me.uk/rpi/pigpio/pigpiod.html)

```bash
sudo touch /etc/systemd/system/pigpiod.service
sudo nano /etc/systemd/system/pigpiod.service
```

```
[Unit]
Description=Daemon required to control GPIO pins via pigpio
[Service]
#ExecStart=/usr/local/bin/pigpiod -l -m #Disable alerts
ExecStart=/usr/local/bin/pigpiod -s 1 -m #Disable alerts
ExecStop=/bin/systemctl kill pigpiod
Type=forking

[Install]
WantedBy=multi-user.target
```

```
[Unit]
Description=Execution pendulum auto start script
# After=network.target <-- 不要

[Service]
ExecStart=/usr/bin/python3 pendulum_auto_start_script.py
WorkingDirectory=/home/ubuntu/pendulum_project/pendulum_test

[Install]
WantedBy=multi-user.target
```

```bash
sudo killall pigpiod
sudo systemctl status pigpiod
```

## ROS ノードをすべて終了するとき

- `grep ros`で ros のプロセスを検索
- `grep -v grep`で`grep`自体のプロセスを除外
- `awk '{ print "kill -9", $2 }'`で残った行からプロセスの process id を取得し、kill する

```
ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh
```

## ROS ノード終了&部品の電源をオフ

ROS ノードを停止するコマンドを実行して、cleanup のコードを実行する。

```
source ./cleanup.sh
```
