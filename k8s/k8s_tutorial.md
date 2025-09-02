# Kubernetes Tutorial

This guide provides instructions on how to deploy the ROS 2 Persistent Parameter Server to a Kubernetes cluster.

## Prerequisites

- Kubernetes cluster (minikube, kind, or cloud provider) is available.
- kubectl CLI tool installed.

## Deployment Steps

### ConfigMap

As an example to load the default parameters to the parameter server, we use configmap here.
This is just an example to describe and load the default parameters to the parameter server.

```console
kubectl apply -f ./k8s/parameters.yaml
```

This command creates the configmap object in the cluster, which can be bound to the parameter server in next step.

> [!NOTE]  
> We can also use the actual yaml file in the host system or shared storage files instead. This depends on the user requirement how to bind the parameter file to the parameter server.

### Deploy Parameter Server

Let's start the parameter server with default parameters provided by the configmap.

```console
kubectl apply -f ./k8s/deployment.yaml
```

### Check Parameters

You can deploy the ROS 2 example talker and listener containers with following pods description.

```console
kubectl apply -f ./k8s/ros2-sample.yaml
```

Now we can see the parameter server is running in the ROS 2 system with specified parameters.

```console
$ kubectl get pods
NAME                                READY   STATUS    RESTARTS   AGE
parameter-server-856cbb6574-qvctl   1/1     Running   0          14m
ros2-listener-7d764d5c8b-jd4vf      1/1     Running   0          34m
ros2-talker-8666f7cf6d-xdqdq        1/1     Running   0          34m

$ $kubectl exec -it ros2-talker-8666f7cf6d-xdqdq -- /bin/bash
root@ros2-talker-8666f7cf6d-xdqdq:/# source /opt/ros/rolling/setup.bash 
root@ros2-talker-8666f7cf6d-xdqdq:/# ros2 param list
/parameter_server:
  a_string
  persistent.a_string
  persistent.pi
  persistent.some_int
  persistent.some_lists.some_integers
  pi
  qos_overrides./parameter_events.publisher.depth
  qos_overrides./parameter_events.publisher.durability
  qos_overrides./parameter_events.publisher.history
  qos_overrides./parameter_events.publisher.reliability
  some_int
  some_lists.some_integers
  start_type_description_service
  use_sim_time
```
