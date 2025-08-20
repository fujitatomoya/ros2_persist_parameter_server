# Deploying the ROS 2 Persistent Parameter Server to Kubernetes

This guide provides instructions on how to deploy the ROS 2 Persistent Parameter Server to a Kubernetes cluster.

## Prerequisites

- A running Kubernetes cluster.
- `kubectl` configured to connect to your cluster.
- Docker installed and configured to build and push images to a container registry.

## 1. Build and Push the Docker Image

First, you need to build the Docker image and push it to a container registry that your Kubernetes cluster can access.

1.  **Build the image:**

    ```bash
    docker build --pull --rm -f ../docker/Dockerfile --build-arg="ROS_DISTRO=rolling" --build-arg="COLCON_WS=/root/colcon_ws" -t <your-docker-registry>/ros2-parameter-server:latest ..
    ```

    Replace `<your-docker-registry>` with the name of your container registry (e.g., `docker.io/your-username`).

2.  **Push the image:**

    ```bash
    docker push <your-docker-registry>/ros2-parameter-server:latest
    ```

## 2. Update the Deployment Manifest

Open the `deployment.yaml` file and replace the placeholder `<your-docker-registry>/ros2-parameter-server:latest` with the actual path to the image you just pushed.

```yaml
# ...
      containers:
      - name: parameter-server
        image: <your-docker-registry>/ros2-parameter-server:latest # IMPORTANT: Replace with your image
# ...
```

## 3. Deploy to Kubernetes

Now you can deploy the parameter server to your Kubernetes cluster using `kubectl`.

1.  **Apply the ConfigMap:**

    ```bash
    kubectl apply -f configmap.yaml
    ```

2.  **Apply the Deployment:**

    ```bash
    kubectl apply -f deployment.yaml
    ```

## 4. Verify the Deployment

You can check the status of the deployment and the running pod:

```bash
kubectl get deployments
kubectl get pods
```

You can also view the logs of the parameter server pod:

```bash
kubectl logs -f <pod-name>
```

Replace `<pod-name>` with the name of the pod from the `kubectl get pods` command.
