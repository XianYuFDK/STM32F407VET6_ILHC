"""Legacy entry point for the real X protocol and debug regression."""
from pathlib import Path
import runpy

runpy.run_path(str(Path(__file__).with_name("test_stepper_x_can.py")), run_name="__main__")
