#!/bin/bash
sudo socat /dev/ttyS0,raw,echo=0 /dev/ttyS1,raw,echo=0
