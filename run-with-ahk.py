from beamiconahk import setup_external_keyboard
import camera
from config import config
from machines.hobby import Hobby, HobbyGcode
import sys

setup_external_keyboard()
        
camera.run_camera_ui(config, machine = Hobby())