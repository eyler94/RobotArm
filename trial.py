#!/usr/bin/env python

import subprocess
import shlex
import numpy as np


np.savetxt('tester.txt', np.array([1, 2, 3, 4]))

first_command = "ls -a"
process = subprocess.Popen(first_command, shell=True)

print("got here")
np.savetxt('tester.txt', np.array([1, 2, 3, 4, 5]))

second_command = "cat tester.txt"
process = subprocess.Popen(second_command, shell=True)

print("Finally got here too")

"""import os
import pprint
import subprocess

command = shlex.split("env -i bash -c 'source init_env && env'")
proc = subprocess.Popen(command, stdout = subprocess.PIPE)
for line in proc.stdout:
  (key, _, value) = line.partition("=")
  os.environ[key] = value
proc.communicate()

pprint.pprint(dict(os.environ))
"""



