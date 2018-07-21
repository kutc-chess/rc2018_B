# ScrpMotor2018 with 超音波センサ

ARRCのMDD(Arduino)のプログラム()に超音波センサ読み取り機能がつきました

* 超音波センサはURM37 V4.0
* 読み取りモードはsensorMode=0 の時、Pulseモード、それ以外の時、PWMモード
* cmd=20 の返り値が超音波センサが測定した距離(cm)となる

## 参考文献
* [サンプルプログラム](https://www.dfrobot.com/wiki/index.php/URM37_V4.0_Ultrasonic_Sensor_(SKU:SEN0001))
