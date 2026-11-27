import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from livox_compat import build


def generate_launch_description():
    return build(1, False)
