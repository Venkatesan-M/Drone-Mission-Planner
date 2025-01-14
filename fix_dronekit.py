import fileinput
import sys
from pathlib import Path

dronekit_init = Path('venv/lib/python3.10/site-packages/dronekit/__init__.py')

with fileinput.FileInput(dronekit_init, inplace=True) as file:
    for line in file:
        # Replace the old import with the new one
        if 'import collections' in line:
            print('from collections import abc as collections')
        else:
            print(line, end='')