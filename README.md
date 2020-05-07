lofi_mqtt

A little program that runs on an Odroid or Raspberry Pi with an attached NRF24l01+.
This program receives packets from my lofi sensor modules and publishes MQTT topics.

Installing on a fresh raspbian image:
1) sudo apt install wiringpi
2) sudo apt install libmosquitto-dev
3) sudo apt install git
4) mkdir projs && cd projs
5) git clone https://github.com/larsenkb/lofi_mqtt.git
6) cd lofi_mqtt
7) make
8) ./lofi_mqtt -lS
