#!/usr/bin/env python
import pexpect

child = pexpect.spawn("unbuffer", ["./Seawolf-8-Software/test_start.sh"])
for line in child:
    if "RESULT: SUCCESS" in line:
        exit(0)
    if "RESULT: FAIL" in line:
        exit(1)
    print(line)
child.close()
