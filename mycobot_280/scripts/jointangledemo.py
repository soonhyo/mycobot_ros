from pymycobot.mycobot import MyCobot
from pymycobot.genre import Angle
from port_setup import setup
mycobot = setup()
mycobot.send_angle(Angle.J2.value, 0, 80)
mycobot.send_angles([0,0,0,0,0,0], 80)
