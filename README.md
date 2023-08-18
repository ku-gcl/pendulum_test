# Overview
倒立振子用のテストコード

# コマンド
## 実行

```
g++ -o LED LED.cpp -lpigpiod_if2 -lrt
sudo ./LED
```

## ROSノードをすべて終了するとき
- `grep ros`でrosのプロセスを検索
- `grep -v grep`で`grep`自体のプロセスを除外
- `awk '{ print "kill -9", $2 }'`で残った行からプロセスのprocess idを取得し、killする

```
ps aux | grep ros | grep -v grep | awk '{ print "kill -9", $2 }' | sh
```

## ROSノード終了&部品の電源をオフ
ROSノードを停止するコマンドを実行して、cleanupのコードを実行する。

```
source ./cleanup.sh
```