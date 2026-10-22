"""Select a test adapter image without changing the user's installed image tags."""
import os
from pathlib import Path
import shutil


def image_path(output, version, image):
    shim=Path(output)/'bin';shim.mkdir()
    docker=shutil.which('docker')
    default='simple-rbt-ros1:noetic' if version=='ros1' else 'simple-rbt-ros2:humble'
    (shim/'docker').write_text('#!/usr/bin/python3\nimport os,sys\nargs=[%r if a==%r else a for a in sys.argv[1:]]\nos.execv(%r,[%r]+args)\n'%(image,default,docker,docker))
    (shim/'docker').chmod(0o700)
    return str(shim)+':'+os.environ['PATH']
