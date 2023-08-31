# Overview

倒立振子用のテストコード

# コマンド

## 実行

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

## ボタン用のスクリプトを実行

```bash
python3 /home/ubuntu/pendulum_project/pendulum_test/pendulum_auto_start_script.py
```

`python`ではなく、`python3`を使用すること。
`python`を使うと、GPIO ピンの ON/OFF が動作しなくなる。


## 電源ライトを点滅させる

```bash
sudo nano /boot/firmware/config.txt
```

config.txtを編集

```config.txt

...

# turn power LED into heartbeat
dtparam=pwr_led_trigger=heartbeat

```

再起動

```bash
sudo reboot
```


## ctrl+Cやctrl+Zのコマンド操作を検知

[Cで強制終了を回避(出来るだけ)](https://qiita.com/Ki4mTaria/items/838f81d3eecd5cc7d91e)


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
