# 🚗 Scenario Detection Webserver – STM32 WiFi Projekt

Dieses Projekt implementiert eine webbasierte Szenario-Erkennung mit einem **STM32L475 IoT Board** und dem integrierten **ES-WiFi Modul**. Ziel ist die Detektion kritischer Fahrzustände wie eine **nicht mehr vermeidbare Kollision**. Über einen einfachen HTTP-Server können die erkannten Szenarien sowie Sensordaten wie Abstand, Geschwindigkeit, Beschleunigung und Gyroskopwerte visualisiert werden.

# In main.c müssen folgende constanten gestezt werden:
#define SSID "WLAN-070011"
#define PASSWORD "xxx"

