#!/usr/bin/python3
import sys

import tools.si5344_lib as siLib
from smbus import SMBus
import time

            

if __name__ == "__main__":
    # print ("Hello")
    if (len(sys.argv)==2):
        reg_filename=sys.argv[-1]
        print("Config Filename Selected:",reg_filename)
    else:
        print("Please provide config file as the single argument")
        exit()

    addrs,vals = siLib.read_config(reg_filename)
    i2cbus = SMBus(1)
    i2caddress = 0x69
    si5344 = siLib.si5344_i2c(i2cbus,i2caddress)

    for i,(addr,val) in enumerate(zip(addrs,vals)):
        si5344.transfer(addr,val)
        if i == 2: time.sleep(1)











