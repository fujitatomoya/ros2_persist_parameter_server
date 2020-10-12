#!/usr/bin/python3

"""A script to start `Server && Client` throught launch file and responsible for killing the Server"""
from threading import Thread
import time 
import os 
import sys
import signal 
import psutil

signal.signal(signal.SIGINT, signal.SIG_DFL)
sleep_time = 3
launchCmd = 'ros2 launch ros2_persistent_parameter_server_test test.launch.py'

def kill_server():
    try:
        time.sleep(sleep_time)
        print("parameter server is about to be killed...")
        program_name = 'server'
        for process in psutil.process_iter():
            if process.name() == program_name:
                path = psutil.Process(process.pid)
                if "install/parameter_server/lib/parameter_server" or "parameter_server/server" in path.exe():
                    os.kill(int(process.pid), signal.SIGINT)
                    print("parameter server is killed successfully")
                    break
    except:
        print("parameter server cannot be killed")
        return
    time.sleep(5)
    print("Press CTRL-C to shutdown...")

t = Thread(target = kill_server, args = ())
t.start()
os.system(launchCmd)
