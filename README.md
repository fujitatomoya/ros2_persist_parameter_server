# ROS2 Persistent Parameter Server
<!-- TOC -->

- [ROS2 Persistent Parameter Server](#ros2-persistent-parameter-server)
- [Background](#background)
- [Overview](#overview)
    - [Persistent Parameter Registration](#persistent-parameter-registration)
        - [Persistent Prefix](#persistent-prefix)
        - [Scope Overview](#scope-overview)
    - [Configurable Options](#configurable-options)
    - [Sequence](#sequence)
- [Getting Started](#getting-started)
    - [Dependent Packages](#dependent-packages)
    - [Prerequisites](#prerequisites)
    - [Build](#build)
    - [Run](#run)
- [Use Client Demo](#use-client-demo)
    - [Build](#build-1)
    - [Run](#run-1)
- [Authors](#authors)
- [License](#license)

<!-- /TOC -->
This is the PoC project for ROS2 Persistent Parameter Server, that resides in the ROS2 system to serve the parameter daemon. The other nodes(e.g the client demo provided in the code) can write/read the parameter in Parameter Server, and ***Parameter Server is able to store the parameter into the persistent storage which user can specify such as tmpfs, nfs, or disk.***


# Background

The disussion is opened [here](https://discourse.ros.org/t/ros2-global-parameter-server-status/10114/13), and centralized parameter server is not a good affinity to ROS2 distributed system architecture. One of the most valuable things about ROS APIs is that we make sure that the messages have specific semantic meaning so that they can’t be misinterpreted. As we develop the ROS 2 tools and best practices we should make sure to bring that same level of rigor to parameters too for greater reusability and correctness.

Although, it is expected to be the following requirement.

- Global configuration that many nodes share (e.g. RTOS priorities, vehicle dimensions, …)
- Generic ROS2 system property server.
- Persistent storage support to re-initialize the system. parameters are modified in runtime and cache it into persistent volume as well. and next boot or next re-spawn, modified parameter will be loaded at initialization. (parameter lifetime is dependent on use case, sometimes system lifetime, sometimes node lifetime.)
- Using ROS1 based application with Parameter Server.

# Overview

![overview_architecture](./images/overview_architecture.png)

Generally ROS2 Parameter Server is simple blackboard to write/read parameters on that. The other nodes can write/read the parameter on the server to share them in the ROS2 system. there is a new concept for "Persistent Parameter" which is described later.

ROS2 Parameter Server is constructed on ROS parameter API's, nothing specific API's are provided to connect to the server from the client. Also, about the security it just relies on ROS2 security aspect.

## Persistent Parameter Registration

### Persistent Prefix

persistent parameter must have prefix ***"persistent"***

### Scope Overview

parameter server has the following scope for persistent parameter. since parameter server is built on top of ROS2 Parameter API, parameter server supports "persistent" parameter based on **/parameter_events** topic.

|  Category  |  Supported  |  Description  |
| ---- | ---- | ---- |
|  Parameter API  |  YES  |  ROS2 Parameter Client API supported, since this activity can be detected via **/parameter_events**.  |
|  Persistent Parameter File  |  YES  | parameter server dedicated argument to specify the file to load as parameters. in addition, all of the persistent parameters will be stored into this file during shutdown.<br> e.g) --file-path /tmp/parameter_server.yaml |
|  Parameter Arguments  |  NO  |  e.g) --ros-args -p persistent.some_int:=42<br>some_int cannot be registed as persistent parameter, since this cannot be notified via **/parameter_events** to parameter server.  |
|  Parameter File Arguments  |  NO  |  e.g) --ros-args --params-file ./parameters_via_cli.yaml<br>same with parameter arguments, cannot be registed as persistent parameter, since these cannot be notified via **/parameter_events**  to parameter server. |
|  Launch Parameter  |  NO  |  e.g) ros2 launch parameter_server parameter_server.launch.py<br>same with parameter arguments, cannot be registed as persistent parameter, since these cannot be notified via **/parameter_events**  to parameter server.  |

## Configurable Options

- Node Name
  Since ROS2 parameter is owned by node, node name will be needed to access the parameters, this is designed to clearify semantics for the parameters and owners. Node name will be "parameter_server" if node name is not specifies. so the other nodes can use "parameter_server" as well to access in the same system Parameter Server. If there must exist multiple parameter servers, these parameter servers need to specify a different node name, such as "parameter_server_[special_string]", please notice that ROS2 node name can only contains alphanumerics and '_'.
- Persistent Volume
  Definition of "Persistent" is different from user and use cases, so it should be configurable to set the path to store the persistent p--file-path FILE_PATHarameter. Expecting if the parameter's lifespan is system boot, path would be "/tmp" because user wants a fresh start via reboot. Or maybe physical persistent volume is chosen if users want to keep the parameter into the hardware storage. At the initialization time, Parameter Server will load the parameters from the storage which is specified by user.
- Node Options
  there are two important options,
  allow_undeclared_parameters: (default true)
  automatically_declare_parameters_from_overrides: (default true)

all of the configuration options will be passed via arguments as followings.

<table>
    <thead>
        <tr>
            <th>Options</th>
            <th>CLI</th>
            <th>Description</th>
        </tr>
    </thead>
    <tbody>
        <tr>
            <td>Node Name</td>
            <td>--ros-args --remap __node:=NODENAME</td>
            <td>in default, "parameter_server" will be used.</td>
        </tr>
        <tr>
            <td>Help</td>
            <td>--help</td>
            <td>show usage.</td>
        </tr>
        <tr>
            <td>File Path</td>
            <td>--file-path FILE_PATH</td>
            <td>in default, "/tmp/parameter_server.yaml" will be used. if specified, that path will be used to store/load the parameter yaml file.</td>
        </tr>
        <tr>
            <td rowspan=2>Node Options</td>
            <td>--allow-declare true/false</td>
            <td>default enabled, if specified allow any parameter name to be set on parameter server without declaration by itselt. Otherwise it does not.</td>
        </tr>
        <tr>
            <td>--allow-override true/false</td>
            <td>default enabled, if specified true iterate through the node's parameter overrides or implicitly declare any that have not already been declared.</td>
        </tr>
    </tbody>
</table>

## Sequence

1. parameter server is initialized via __params:=<xxx.yaml>
   this is just a initial parameter(not persistent) to load into the parameter server's memory.
2. parameter server loads parameter specified yaml file via --file-path.
   and then parameter server will overwrite or declare parameters.
   (*) at #1 parameters might be overwritten.
3.  parameter server starts the main loop with callback for parameter changes.
4.  if the parameter changes are on "/persistent" that will be stored in storage at this time.
5.  at the finilization, flash all of the "/persistent" parameters into the file system.

# Getting Started

## Dependent Packages

```
apt install libyaml-cpp-dev
apt install libboost-program-options1.65-dev libboost-filesystem1.65-dev
```

## Prerequisites

currently verified only Ubuntu18.04.

ros2 source build environment([Linux-Development-Setup/](https://index.ros.org/doc/ros2/Installation/Dashing/Linux-Development-Setup/)) is required to build and run the parameter server.

## Build

to install local colcon workspace,

```
# cd <colcon_workspace>/src
# git clone https://github.com/fujitatomoya/ros2_persist_parameter_server
# cd <colcon_workspace>
# colcon list
# colcon build
# source install/local_setup.bash
```

## Run

1. start parameter server.

```
# cp <colcon_workspace>/src/ros2_persist_parameter_server/server/param/parameter_server.yaml /tmp/

# ros2 run parameter_server server
[INFO] [parameter_server]: Parameter Server node named: '/parameter_server' started and ready, and serving '9' parameters already!
...<snip>

```

2. update persistent parameter.

```
# ros2 param set /parameter_server persistent.some_int 81
Set parameter successful
# ros2 param set /parameter_server persistent.a_string Konnichiwa
Set parameter successful
# ros2 param set /parameter_server persistent.pi 3.14159265359
Set parameter successful
# ros2 param set /parameter_server persistent.some_lists.some_integers 81,82,83,84
Set parameter successful
```

3. restart parameter server.

```
# ros2 run parameter_server server
[INFO] [parameter_server]: Parameter Server node named: '/parameter_server' started and ready, and serving '9' parameters already!
...<snip>
```

4. check persistent parameter is precisely cached and loaded into parameter server.

```
# ros2 param get /parameter_server persistent.a_string
String value is: Konnichiwa
# ros2 param get /parameter_server persistent.pi
Double value is: 3.14159265359
# ros2 param get /parameter_server persistent.some_int
Integer value is: 81
# ros2 param get /parameter_server persistent.some_lists.some_integers
String value is: 81,82,83,84
```

# Use Client Demo

The client demo verifies the following functions

 - persistent parameter can be read/stored to/from the file system.
 - persistent parameter can be read/modified from parameter client.
 - non-persistent parameter cannot be read/stored to/from the file system.
 - non-persistent parameter can be read/modified from parameter client

!!!NOTE The client demo provided does not support the use of the specified parameter name, it only demonstrates and verifies the basic functions of the parameter server.

## Build
The build steps are similar to the above build step, except that you need to install ros2 launch package with `respawn` feature support, which is only support in master branch now.

```
# cd <launch_workspace>/src
# git clone https://github.com/ros2/launch.git
# cd <launch_workspace> && colcon build
# cd <colcon_workspace>/src
# git clone https://github.com/fujitatomoya/ros2_persist_parameter_server
# cd <colcon_workspace>
# source <path-to-ros2_distro>/install/setup.bash   # ros2 eloquent or ros2 foxy was supported.
# colcon build
# source install/local_setup.bash
# source <launch_workspace>/install/setup.bash
```

## Run

start.py is the entry for test.
start.py will call test.launch.py file to start persistent parameter server and the test client, it also creates a thread to kill parameter server after specified time. All function tests are finished in client.

!!!NOTE After the test is completed, you need to press `ctrl-c` to stop process manually.

```
# cd <colcon_workspace>/src/ros2_persist_parameter/client_demo
# ./start.py
```

The phenomenon as below

!!!NOTE Client has a 5-seconds sleep during server restarts.

```
......   // omit some output logs

[ros2-2] [INFO] [1601447662.145760479] [client]: ***************************************************************************
[ros2-2] [INFO] [1601447662.145794365] [client]: *********************************Test Result*******************************
[ros2-2] [INFO] [1601447662.145817265] [client]: a. Read Normal Parameter                                     :             PASS
[ros2-2] [INFO] [1601447662.145842530] [client]: b. Read Persistent Parameter                                 :             PASS
[ros2-2] [INFO] [1601447662.145863430] [client]: c. Modify Existed Normal parameter                           :             PASS
[ros2-2] [INFO] [1601447662.145885082] [client]: d. Modify Existed Persistent parameter                       :             PASS
[ros2-2] [INFO] [1601447662.145906067] [client]: e. Add New Normal parameter                                  :             PASS
[ros2-2] [INFO] [1601447662.145926790] [client]: f. Add New Persistent parameter                              :             PASS
[ros2-2] [INFO] [1601447662.145948146] [client]: g. Test Normal Parameter Not Stores To File                  :             PASS
[ros2-2] [INFO] [1601447662.145969623] [client]: h. Test Persistent Parameter Stores To File                  :             PASS
[ros2-2] [INFO] [1601447662.145990707] [client]: i. Test New Added Normal Parameter Not Stores To File        :             PASS
[ros2-2] [INFO] [1601447662.146011312] [client]: j. Test New Added Persistent Parameter Stores To File        :             PASS
```

# Authors

* **Tomoya Fujita** --- Tomoya.Fujita@sony.com

# License

Apache 2.0
