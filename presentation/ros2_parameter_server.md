---
marp: true
theme: default
header: "__ROS 2 Persistent Parameter Server__"
footer: "[fujitatomoya@github](https://github.com/fujitatomoya)"
---

## [ROS 2 Persistent Parameter Server](https://github.com/fujitatomoya/ros2_persist_parameter_server)

![bg right:35% width:300px](../images/QR.png)

- inspired by ROS 1 parameter server.
- can set/get any parameters in this global server.
- can save/load the parameters in storage.

<!---
Comment Here
--->

---

![bg 70%](https://images.squarespace-cdn.com/content/v1/606d378755a86f589aa297b7/1653397531343-6M4IQ4JWDQV1SQ8W17UN/HumbleHawksbill_TransparentBG-NoROS.png)
![bg 75%](https://images.squarespace-cdn.com/content/v1/606d378755a86f589aa297b7/ebf9b1d5-45b7-4a73-8f48-dc5d3f4fc8fc/JazzyJalisco_Final.png?format=2500w)
![bg 90%](https://www.therobotreport.com/wp-content/uploads/2025/05/kilted-Kaiju-featured.jpg)
![bg 70%](https://images.squarespace-cdn.com/content/v1/606d378755a86f589aa297b7/1628726028642-TVRVRIQL914IVYWV8MG9/rolling.png)

<!---
Supported platforms
--->

---

## Why we need this?

- Global configuration that many nodes share (e.g. RTOS priorities, vehicle dimensions, …)
- Generic ROS 2 system or localhost wide parameter server.
- <span style="color:red;">Persistent storage support to re-initialize the system.</span>
  - **parameters are modified in runtime and cached into persistent volume as well. and next boot or next re-spawn, modified parameters will be loaded at initialization. (parameter lifetime is dependent on use case, sometimes system lifetime, sometimes node lifetime.)**
- Using ROS 1 based application with Parameter Server.

<!---
Comment Here
--->

---

![bg 90%](../images/overview_architecture.png)

---

## How to Run

- Docker

```console
$ docker run -it tomoyafujita/ros2_param_server:rolling /bin/bash
root@bf4d904e3800:~/colcon_ws# ros2 run persist_parameter_server server
```

- Kubernetes

```console
$ kubectl apply -f ./k8s/parameters.yaml
$ kubectl apply -f ./k8s/deployment.yaml
```

<!---
Comment Here
--->

---

## Issues and PRs are always welcome 🚀

https://github.com/fujitatomoya/ros2_persist_parameter_server

![bg left:35% width:300px](../images/QR.png)

<!---
Comment Here
--->
