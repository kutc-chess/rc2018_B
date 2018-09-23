# ScrpMotor2018 with 発射・装填
ARRCのMDD(Arduino)のプログラムに発射・装填機能がつきました   

* cmd = 10, data = 1 で発射できる(shootable = true)なら発射し1を返す, 出来なければ0を返す
* cmd = 10, data = 0 で発射できる状態かどうかを上記の通り返す
