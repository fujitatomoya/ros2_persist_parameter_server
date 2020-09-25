#!/usr/bin/python3

"""A script to start Server && Client throught launch file and to control the restart of Server"""
from threading import Thread
import time 
import os 
import sys
import signal 
import psutil

signal.signal(signal.SIGINT, signal.SIG_DFL)
sleep_time = 3
launchCmd = 'ros2 launch client_demo test.launch.py'

def restart_server():
    try:
        time.sleep(sleep_time)
        print("parameter server restart now...")
        program_name = 'server'
        for process in psutil.process_iter():
            if process.name() == program_name:
                os.kill(int(process.pid), signal.SIGINT)
                print("parameter server restart successfully")
                break
    except:
        print("parameter server restart failed")
        return

t = Thread(target = restart_server, args = ())
t.start()

os.system(launchCmd)
