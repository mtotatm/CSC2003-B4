from m5stack import *
from m5ui import *
from uiflow import *
from m5mqtt import M5mqtt
import json



setScreenColor(0x111111)


getdata = None
counter = None







uart1 = machine.UART(1, tx=33, rx=32)
uart1.init(9600, bits=8, parity=None, stop=1)
setScreenColor(0xffff00)
m5mqtt = M5mqtt('m5stickcplus', 'thingsboard.cloud', 1883, 'testing', 'password', 300)
m5mqtt.start()
counter = 0
getdata = ' '
while True:
  if uart1.any():
    counter = counter + 1
    getdata = (uart1.read()).decode()
    if getdata != ' ':
      m5mqtt.publish(str('v1/devices/me/telemetry'), str((json.dumps(({counter:getdata})))), 1)
    setScreenColor(0x33cc00)
  wait_ms(2)
