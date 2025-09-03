---
marp: true
theme: default
header: "__ROS 2 Persistent Parameter Server__"
footer: "[fujitatomoya@github](https://github.com/fujitatomoya)"
---

## [ROS 2 Persistent Parameter Server](https://github.com/fujitatomoya/ros2_persist_parameter_server)

![bg right:35% width:300px](../../images/QR.png)

- ROS 1 parameter serverからインスピレーションを得て開発。
- このグローバルサーバーで任意のパラメータを設定/取得可能。
- パラメータをストレージに保存/読み込み可能。

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

- 多くのノードが共有するグローバル設定（例：システム設定や構成など）
- 汎用的なROS 2システムやlocalhost全体のパラメータサーバー。
- <span style="color:red;">永続化ストレージによるシステムの再初期化サポート。</span>
    - **パラメータは実行時に変更され、永続化ボリュームにキャッシュされます。次回起動や再起動時には、変更されたパラメータが初期化時に読み込まれます。（パラメータの寿命はユースケースに依存し、システム寿命の場合もあれば、ノード寿命の場合もあります。）**
- Parameter Serverを使用するROS 1ベースのアプリケーションとの互換性。

<!---
Comment Here
--->

---

![bg 90%](../../images/overview_architecture.png)

---

## How to Run

- Docker

```console
$ docker run -it tomoyafujita/ros2_param_server:rolling /bin/bash
root@bf4d904e3800:~/colcon_ws# ros2 run parameter_server server
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

![bg left:35% width:300px](../../images/QR.png)

<!---
Comment Here
--->

