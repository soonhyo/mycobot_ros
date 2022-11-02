from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from port_setup import setup
import time
mycobot = setup()
for i in range(1, 7):
    ret = mycobot.release_servo(i)
    print(ret)
    print(mycobot.is_servo_enable(i))
    time.sleep(1)
# mycobot.release_servo(3)
