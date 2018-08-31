# ScrpMotor2018 with 超音波センサ

ARRCのMDD(Arduino)のプログラム()に超音波センサ読み取り機能がつきました

* 超音波センサはURM37 V4.0
* 読み取りモードはsensorMode=0 の時、Pulseモード、それ以外の時、PWMモード
* cmd=20 の返り値が超音波センサが測定した距離(cm)となる

## ピン配置
* Arduino -> URM V4.0
* Vcc -> Pin 1 Vcc
* GND -> Pin 2 GND
* Pin 9 (出力) -> Pin 6 COMP/TRIG
* Pin 3 (入力) -> Pin 4 ECHO (If you want to read by Pulse. PWMで読む時は使いません。)
* Pin A0 (入力) -> Pin 7 DAC (If you want to read by PWM. Pulseで読む時は使いません。)

## 参考文献
* [サンプルプログラム](https://www.dfrobot.com/wiki/index.php/URM37_V4.0_Ultrasonic_Sensor_(SKU:SEN0001))
