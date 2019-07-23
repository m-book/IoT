IoT
====

## Description
人がそこに来た時間と離れた時間を記録する

## Requirement
* Python3
* MySQL(8.0.16)
* M5Stack
* サーマルカメラ

## Usage
#### M5Stackのセットアップ
* Arduino.ioからHumanSensor.ioの実行

#### IoTServerの起動
###### MySQLのセットアップ
* cd IoT_Server
初回起動時のみ
* # self.mysql.create_table()のコメントアウト解除
共通
* python IoTserver.py
*IoTServerを先に起動しておかないとM5Stack側で表示がおかしくなるかもしれません*
## Install
* git clone git@github.com:m-book/IoT.git.git
