#!/usr/bin/env python

# -*- coding: utf-8 -*-

import os
import time
import threading
from ctypes import *



def fun_timer():
    global timer
    timer = threading.Timer(0.05, fun_timer)
    timer.start()
    lib.send_cmd(vel,angle)


if __name__=="__main__":

    vel = 1500
    angle = 1500
    lib_path = os.path.abspath(os.path.join(os.getcwd(), "..")) + "/lib"+ "/libart_driver.so"
    so =  cdll.LoadLibrary
    lib = so(lib_path)
    #print lib
 
    try:
        car = "/dev/ttyUSB0"
        if(lib.art_racecar_init(38400,car) < 0):
            raise 
            pass 

        timer = threading.Timer(0.05, fun_timer)
        timer.start()
        
        
        while(1):

            pass
                
                
               

    except:
        print "error"

    finally:
        print "finally"