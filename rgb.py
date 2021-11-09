# Bluetooth RGB Light
#
# Use nRFConnect app from the App store or an app, connect to the Nano and write a color to control the LED.

import bluetooth
import random
import struct
import time
from ble_advertising import advertising_payload
from machine import Pin, PWM, ADC
from micropython import const

LED_PIN_R = 15
LED_PIN_G = 16
LED_PIN_B = 17

_IRQ_CENTRAL_CONNECT = const(1)
_IRQ_CENTRAL_DISCONNECT = const(2)
_IRQ_GATTS_WRITE = const(3)

_FLAG_READ = const(0x0002)
_FLAG_WRITE = const(0x0008)
_FLAG_NOTIFY = const(0x0010)
_FLAG_INDICATE = const(0x0020)

_SERVICE_UUID = bluetooth.UUID(0x1523)
_LED_CHAR_UUID = (bluetooth.UUID(0x1525), _FLAG_WRITE)
_LED_SERVICE = (_SERVICE_UUID, (_LED_CHAR_UUID,),)

_PWM_FREQ = 1000

class BLERGBLight:
    def __init__(self, ble, pwm_r, pwm_g, pwm_b, name="NANO RP2040"):
        self.pwm_r = pwm_r
        self.pwm_g = pwm_g
        self.pwm_b = pwm_b
        self._ble = ble
        self._ble.active(True)
        self._ble.irq(self._irq)
        ((self._handle,),) = self._ble.gatts_register_services((_LED_SERVICE,))
        self._connections = set()
        self._payload = advertising_payload(name=name, services=[_SERVICE_UUID])
        self._advertise()

    def _irq(self, event, data):
        # Track connections so we can send notifications.
        if event == _IRQ_CENTRAL_CONNECT:
            conn_handle, _, _ = data
            self._connections.add(conn_handle)
            print('device connected')
        elif event == _IRQ_CENTRAL_DISCONNECT:
            conn_handle, _, _ = data
            self._connections.remove(conn_handle)
            # Start advertising again to allow a new connection.
            print('device disconnected')
            self._advertise()
        elif event == _IRQ_GATTS_WRITE:
            print("incoming BLE data")
            print((self._ble.gatts_read(data[-1])))
            rec_duty_r = 255-int(self._ble.gatts_read(data[-1])[0])
            rec_duty_g = 255-int(self._ble.gatts_read(data[-1])[1])
            rec_duty_b = 255-int(self._ble.gatts_read(data[-1])[2])

            print(rec_duty_r)
            print(rec_duty_g)
            print(rec_duty_b)

            duty_r = int(65000*(rec_duty_r / 255))
            duty_g = int(65000*(rec_duty_g / 255))
            duty_b = int(65000*(rec_duty_b / 255))

            print(duty_r)
            print(duty_g)
            print(duty_b)

            self.pwm_r.freq(_PWM_FREQ)
            self.pwm_g.freq(_PWM_FREQ)
            self.pwm_b.freq(_PWM_FREQ)

            self.pwm_r.duty_u16(duty_r)
            self.pwm_g.duty_u16(duty_g)
            self.pwm_b.duty_u16(duty_b)



    def _advertise(self, interval_us=500000):
        self._ble.gap_advertise(interval_us, adv_data=self._payload)

if __name__ == "__main__":
    ble = bluetooth.BLE()

    pwm_r = PWM(Pin(15))
    pwm_g = PWM(Pin(16))
    pwm_b = PWM(Pin(17))

    led_ble = BLERGBLight(ble, pwm_r, pwm_g, pwm_b )

    while True:
        time.sleep_ms(1000)

