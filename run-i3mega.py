import serial
import camera
from config import config
import sys

from machines.anycubic_i3mega_rolf import AnycubicI3Mega

# 250000 
# ser = serial.Serial('COM3', baudrate= 250000, timeout=5)
# # gcode = open("path/to/gcode/file.gcode", "r")

# # for line in gcode:
# ser.write("G91 G1 X10 F100".encode())
#     # response = ser.readline().decode()
#     # print(response)

# ser.close()

# class Machine:

#     def __init__(self):
#         self.gcode = HobbyGcode()


#     def move_xyz(self, *args, **kwargs):
#         code = self.gcode.move_xyz(*args, **kwargs)
#         print(code)
#         beamiconahk.Command("MDI", code)

#     def workingarea(self):
#         f = config["machines"][0]
#         return {
#             "x": [0,   f["maxx"]],
#             "y": [0, - f["maxy"]],
#             "z": [0, - f["maxz"]],
#         }
        
machine = AnycubicI3Mega()
print("camera")
camera.run_camera_ui(config, machine = machine)