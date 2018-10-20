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

## 旧ScrpMotorとの相違点
  * 2つのモーターから3つのモーターへ
  * 周波数を変更・設定
  * シリアル通信の送受信切り替えピンを変更
  * 減速コマンドを削除
  * LEDピンの設定を変更

## 現ScrpMotorとの相違点
  * 超音波センサに対応(Analog)
  * テープLEDに対応(10, 11)

## テープLED
```cpp
slave.addCMD(30, changeLEDFlag);
```
 * テープLEDの光らせ方を変更するやつです。
 * 1ビット目(0b0000X)はゾーンの設定をします。
 	* 1で赤ゾーン、0で青ゾーンです。
 * 2ビット目(0b000X0)は投げる腕の設定をします。
 	* 1で左側、0で右側です。
 * 3ビット目(0b00X00)は目標テーブルの設定をします。
 	* 1で2.4m、0でそれ以外です。
 * 4ビット目(0b0X000)はLEDの消灯方法の設定をします。
 	* 1で徐々に消え(ワイプ)、0で一斉に消えます。
 * 4ビット目(0bX0000)は実行フラグの設定をします。
 	* 1で実行、0で停止です(実行中にこのフラグを0にすると強制終了します)。

 ```cpp
 slave.addCMD(31, changeLEDWiping);
 ```
 	* テープLEDのワイプにかかる時間を設定します(ms)。
 	* ここで設定した時間をLEDの個数で割ったものが、ワイプの間隔になります(デフォルトだと50 ms)。

 ```cpp
 slave.addCMD(32, changeLEDShining);
 ```
 	* テープLEDのワイプが終わってから光る時間を設定します(ms)。
 	* ワイプが終わってから消灯が始まるまでの間の時間です。

## ハード仕様
* Raspberry PiがMaster, Arduino Pro miniがSlave
* Arduino Pro miniのUARTをRS485に変換している。(Raspberry Piも同様)

