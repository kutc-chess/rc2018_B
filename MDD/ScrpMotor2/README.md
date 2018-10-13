# ScrpMotor2018

ARRCのMDD(Arduino)の標準プログラムです。CMDとMDの対応は以下の通りです。   
追記
MDの仕様が変更されたので周波数の変更

| CMD | ピン | 周波数(Hz) |
|:----:|:----:|----:|
| 2 | 5, 6 | 976.5625 |
| 3 | 10, 11 | 488.48125 |
| 3 | 3, 9 | 488.48125 |

## ライブラリ
[Arduino IDE](https://www.arduino.cc/en/main/software)で、windowsなら「ドキュメント/Arduino/libraries/」におけばいいです。

[ScrpSlave](https://github.com/Kitasola/ScrpMotor2018/tree/master/ScrpSlave)
> 偉大なる大先輩に感謝

[Utility](https://github.com/EHbtj/Arduino-Utility)
> 使い方は[こっち](https://ehbtj.com/electronics/arduino-utility-library/)のほうが分かりやすい

## 旧ScrpMoterとの相違点
  * 2つのモーターから3つのモーターへ
  * 周波数を変更・設定
  * シリアル通信の送受信切り替えピンを変更
  * 減速コマンドを削除
  * LEDピンの設定を変更

## ハード仕様
* Raspberry PiがMaster, Arduino Pro miniがSlave
* Arduino Pro miniのUARTをRS485に変換している。(Raspberry Piも同様)

