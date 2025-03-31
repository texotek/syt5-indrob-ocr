# Motor Control Scripts

Die Motor Controler Scripts haben das Ziel den Arduino ESP32 als Motorsteuerung zu verwenden.
Über eine H-Brücke (L298N) [1]

Fuer das Programmieren des Arduino ESP32-S3 wurde das Espressif ESP-IDF[2] genutzt.

Es erlaubt mir das Programmieren des ESPs ohne Arduino Software, die ich vermeiden wollte.
Fuer das Aufsetzen von ESP-IDF kann man sich [3] als startpunkt nehmen.

Kompilieren dann mit:
```
idf.py set-target esp32-s3
idf.py build
```

Dann das flashen auf den Mikrocontroller:
```
idf.py flash -p /dev/<tty> && idf.py monitor
```

Danach started die `app_main` im Mikrocontroller und die Motoren fangen an sich zu bewegen.
Funtionalität ist in der [motor_control.c](./main/motor_control.c).

# Resources
[1] - https://www.handsontec.com/dataspecs/module/L298N%20Motor%20Driver.pdf
[2] - https://docs.espressif.com/projects/esp-idf/en/stable/esp32/index.html
[3] - https://docs.espressif.com/projects/esp-idf/en/stable/esp32/get-started/linux-macos-setup.html

