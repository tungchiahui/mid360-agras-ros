import os
import sys

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from mid360_common import build


def generate_launch_description():
    return build(4, False)
