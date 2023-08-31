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
g++
```

## ボタン用のスクリプトを実行

```bash
python3 /home/ubuntu/pendulum_project/pendulum_test/shutdown.py
```

`python`ではなく、`python3`を使用すること。
`python`を使うと、GPIO ピンの ON/OFF が動作しなくなる。

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
