# Docker 環境に eR@sers Kachaka をインストールする方法

> [!NOTE]
> このドキュメントは `Ubuntu 22.04` 以外のディストリビューション環境や，ローカル環境に変更を加えたくないユーザーにむけた，Docker を用いて eR@sers Kachaka をセットアップする手順を解説します．

## 推奨環境

|||
|:---:|:---|
|OS|Ubuntu|
|CPU ARCH|$\text{x86}$|
|RAM|$\text{8GB} \ge$|

## Docker をインストールする

1. 以下のコマンドを実施して Docker をインストールします．   
    ```bash
    sudo apt update && sudo apt install -y ca-certificates curl gnupg lsb-release &&\
    sudo mkdir -p /etc/apt/keyrings &&\
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg &&\
    echo   "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
      $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null &&\
    sudo apt update && sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin &&\
    sudo usermod -aG docker $USER
    ```
1. 次のコマンドを実施して `docker` グループに所属するよう既存セッションに反映させます．
    ```bash
    newgrp docker
    ```
1. 以下のコマンドを実施して `docker` コマンドが利用できることを確認してください．   
    ```bash
    docker images
    ```

## eR@sers Kachaka をダウンロードする
1. 次のコマンドを実施して任意の場所に eR@sers Kachaka をクローンします．
    ```bash
    git clone -b devel/nakatogawa https://github.com/trcp/erasers_kachaka.git
    ```

## Bridge イメージをビルドする
1. `erasers_kachaka` ディレクトリに移動する
    ```bash
    cd path/to/erasers_kachaka
    ```
1. 次のコマンドを実施して２つの Kachaka 通信用のブリッジ Docker イメージをビルドしてください．
    - 公式 Docker イメージをビルドする
        ```bash
        docker compose build official_bridge
        ```
    - eR@sers kachaka 改造 Docker イメージをビルドする
        ```bash
        docker compose build nomap_bridge
        ```
    
    > PC のスペックによってビルド時間は左右されますが，ビルド完了にはそれぞれ５分ほどかかります．

## eR@sers Kachaka イメージをビルドする
1. `erasers_kachaka` ディレクトリに移動する
    ```bash
    cd path/to/erasers_kachaka
    ```
1. 次のコマンドを実行してコンテナ内パスワードを設定してください．
    ```bash
    export PASSWORD=password
    ```
1. 次のコマンドを実施して eR@sers Kachaka イメージをビルドします．
    ```bash
    docker compose build erasers_kachaka
    ```
    > PC のスペックによってビルド時間は左右されますが，ビルド完了には10分ほどかかります．

## 環境変数を定義する
1. `~/.bashrc` に次の環境変数らを追記してください．これらの変数は eR@sers Kachaka の各種設定に使用します．
    ```bash
    # kachaka
    export KACHAKA_NAME="er_kachaka"
    export KACHAKA_IP=192.168.234.102
    export USE_TOF_POINTS=True
    export USE_RVIZ=True
    export BRINGUP_TYPE=0
    export USE_SHELF=True
    export SHELF_TYPE=0
    export KACHAKA_ERK_PATH=~/colcon_ws/src/erasers_kachaka
    ## DO NOT EDIT !!!!
    export GRPC_PORT=26400 
    export API_GRPC_BRIDGE_SERVER_URI="${KACHAKA_IP}:${GRPC_PORT}"

    # ROS
    export ROS_DOMAIN_ID=1
    export ROS_LOCALHOST_ONLY=0
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ```
    > - `KACHAKA_IP`
    > - `ROS_DOMAIN_ID`
    > 
    > これらの環境変数は適宜変更する必要があります．

1. 次のコマンドを実施して `~/.bashrc` を再読込します．
    ```bash
    . ~/.bashrc
    ```

---

以上で Docker 環境に ROS2 をインストールする手順は完了となります．

- [README にもどる](/README.md)