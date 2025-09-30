#!/usr/bin/python3

"""A script to start `Server && Client` through launch file and responsible for killing the Server"""
from threading import Thread

import os
import psutil
import shutil
import signal
import subprocess
import sys
import time

signal.signal(signal.SIGINT, signal.SIG_DFL)
sleep_time = 3
launchServerCmd = ['ros2', 'launch', 'persist_parameter_server', 'test.launch.py']
launchClientCmd = ['ros2', 'run',
                   'persist_parameter_server', 'client_default']

launchServerCmdWithNodeOptions = [
    'ros2', 'launch', 'persist_parameter_server', 'test.launch.py', 'allow_dynamic_typing:=true']
launchClientCmdWithNodeOptions = [
    'ros2', 'run', 'persist_parameter_server', 'client_with_node_options']

if shutil.which('ros2') is None:
    print("source <colcon_ws>/install/setup.bash...then retry.")
    sys.exit(1)

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
    #print("Press CTRL-C to shutdown...")

# Start Server process with re-spawn enabled, this process stays running
server_process = subprocess.Popen(launchServerCmd, preexec_fn=os.setsid)
print(f"Parameter Server Process started with PID: {server_process.pid}")

# Start test client process
client_process = subprocess.Popen(launchClientCmd)
print(f"Parameter Client Process started with PID: {client_process.pid}")

# Start killer thread to re-spawn the parameter server
t = Thread(target = kill_server, args = ())
t.start()

# Wait until the client process finishes
return_code = client_process.wait()

# Cleanup the process and thread
t.join()
os.killpg(os.getpgid(server_process.pid), signal.SIGTERM)

print("\nTest with default options finished. Proceeding to testing with node options")

# Start the server
server_process = subprocess.Popen(
    launchServerCmdWithNodeOptions, preexec_fn=os.setsid)
print(f"Parameter Server Process started with PID: {server_process.pid}")

# Start test client process
client_process = subprocess.Popen(launchClientCmdWithNodeOptions)
print(f"Parameter Client Process started with PID: {client_process.pid}")

# Wait until the client process finishes and then kill the server
return_code2 = client_process.wait()
os.killpg(os.getpgid(server_process.pid), signal.SIGTERM)

print("\nTest process finished.")
print(f"Return Code: {return_code}")

# Check if the client process completed successfully
if return_code == return_code2 == 0:
    print("The process completed successfully.")
    sys.exit(0)
else:
    print("The process failed.")
    sys.exit(1)
