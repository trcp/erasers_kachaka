# ローカル環境に eR@sers Kachaka をインストールする方法

## 推奨環境

|||
|:---:|:---|
|OS|Ubuntu 22.04|
|CPU ARCH|$\text{x86}$|
|RAM|$\text{8GB} \ge$|
|ROS2 DISTRO|$\text{Humble}$|

## ROS2 Humble をインストールする
1. 次のコマンドを実施し，' Ubuntu Universe repository' が有効になっていることを確認してください．
    ```bash
    sudo apt install -y software-properties-common &&\
    sudo add-apt-repository universe -y
    ```
1. ROS2 Humble をインストールするのに必要な認証キー，リポジトリを以下のコマンドを実施して追加します．
    ```bash
    sudo apt update && sudo apt install curl -y &&\
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F'"' '{print $4}') &&\
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" &&\
    sudo dpkg -i /tmp/ros2-apt-source.deb
    ```
1. 次のコマンドを実施して APT リポジトリを更新します．
    ```bash
    sudo apt update
    ```
1. 次のコマンドを実施して ROS2 Humble をインストールします．インストールには２つのオプションがあります．
    - ROS2 Humble Desktop をインストールする（推奨）
        ```bash
        sudo apt install -y ros-humble-desktop
        ```
    - ROS2 Humble Base をインストールする（最小構成を希望する場合）
        ```bash
        sudo apt install -y ros-humble-ros-base
        ```
1. ROS2 開発用パッケージを以下のコマンドを実施してインストールします．
    ```bash
    sudo apt install -y ros-dev-tools
    ```
1. 次のコマンドを実施して `~/.bashrc` に ROS2 読み込みコマンドを記述します．
    ```bash
    echo ". /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```
1. 次のコマンドを実施して `~/.bashrc` を再読込します．
    ```bash
    . ~/.bashrc
    ```
1. 以下のコマンドを実施して `ros2` コマンドが使用できれば ROS2 のインストールは完了です．
    ```bash
    ros2 topic list
    ```

## Docker をインストールする

> [!NOTE]
Kachaka と通信するためには Docker のインストールが必須となります．

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

## ワークスペースを作成する
　ワークスペースとは ROS2 において開発環境（プログラムや各種パッケージ等）をまとめた１つのディレクトリです．

> [!IMPORTANT]
このドキュメントでは作成するワークスペース名を `~/colcon_ws` とします．既存のワークスペースまたは別名のワークスペースを作成する場合は以降のドキュメント内に記述されているパス `~/colcon_ws` を適宜書き換えてください．

1. 以下のコマンドを実施してホームディレクトリ上に `colcon_ws` という `src` ディレクトリを内包したディレクトリを作成します．
    ```bash
    cd ~ &&  mkdir -p colcon_ws/src
    ```

## eR@sers Kachaka をダウンロードする
1. 以下のコマンドを実施して作成したワークスペースの `src` ディレクトリに移動します．
    ```bash
    cd ~/colcon_ws/src
    ```
1. 次のコマンドを実施して eR@sers Kachaka をクローンします．
    ```bash
    git clone -b devel/nakatogawa https://github.com/trcp/erasers_kachaka.git
    ```

## 依存関係パッケージをクローンする
1. 以下のコマンドを実施して作成したワークスペースの `src` ディレクトリに移動します．
    ```bash
    cd ~/colcon_ws/src
    ```
1. 次のコマンドを実施して eR@sers Kachaka に依存するパッケージをクローンします．
    ```bash
    vcs import . < erasers_kachaka/setup.repos
    ```
1. `ls` コマンドでカレントディレクトリ内に以下のパッケージが追加されていることを確認します．
    - cartographer_ros_kachaka
    - emcl2
    - kachaka-api
    - rclpy_util

## eR@sers Kachaka 依存 Python パッケージをインストールする
1. 以下のコマンドを実施して `pip` をインストールします．
    ```bash
    sudo apt install -y python3-pip
    ```
1. 以下のコマンドを実施して `pip` を最新バージョンに更新します．
    ```bash
    python3 -m pip install --upgrade pip
    ```
1. 以下のコマンドを実施して作成したワークスペースの `src` ディレクトリに移動します．
    ```bash
    cd ~/colcon_ws/src
    ```
1. 以下のコマンドを実施して eR@sers Kachaka に依存する Python パッケージをインストールします．
    ```bash
    pip install -r erasers_kachaka/requirements.txt
    ```
1. 以下のコマンドを実施して `kachaka-api` が正常にインストールされていることを確認してください．
    > コマンド実行時に何もログが表示されなければインストール成功です．

    ```bash
    python3 -c "import kachaka_api"
    ```

## ROS2 パッケージの各種依存関係を解決する
1. 以下のコマンドを実行して `rosdep` を初期化，更新します．
    ```bash
    sudo apt update && sudo rosdep init && rosdep update
    ```
    > もし，次のエラーが発生した場合は次のコマンドを実行してください．
    > `sudo apt update && rosdep update`
    > ```
    > ERROR: default sources list file already exists:
    >     /etc/ros/rosdep/sources.list.d/20-default.list
    > Please delete if you wish to re-initialize
    > ```
1. 以下のコマンドを実施して作成したワークスペースの `src` ディレクトリに移動します．
    ```bash
    cd ~/colcon_ws/src
    ```
1. 以下のコマンドを実行して依存関係をインストールします．
    ```bash
    rosdep install -y -i --from-path .
    ```

    > ネットワーク環境によって時間は変わりますが，インストールの完了には５分ほどかかります．

## ワークスペースをビルドする
1. 次のコマンドを実施してワークスペース直下に移動します．
    ```bash
    cd ~/colcon_ws
    ```
1. 次のコマンドを実施してワークスペースをビルドします．
    - CPU のコア数やメモリが少ない場合以下のコマンドを実施してプロセス制限の下でビルドしてください．
        ```bash
        MAKEFLAGS="-j1 -l1" colcon build --symlink-install --packages-up-to erasers_kachaka_bringup --parallel-workers 1
        ```
    - ある程度のスペックのある PC では以下のコマンドを実施してください．もし以下のコマンドを実施して PC がフリーズした場合上記のコマンドを実施してください．
        ```bash
        colcon build --symlink-install --packages-up-to erasers_kachaka_bringup
        ```
     > PC のスペックによってビルド時間は左右されますが，ビルド完了には３分ほどかかります．
1. ビルド完了後，以下のコマンドを実施して `~/.bashrc` にワークスペースの設定を読み込ませます．
    ```bash
    echo ". ~/colcon_ws/install/setup.bash" >> ~/.bashrc
    ```
1. 次のコマンドを実施して `~/.bashrc` を再読込します．
    ```bash
    . ~/.bashrc
    ```
1. 次のコマンドを実施して `erasers_kachaka_` から始まるパッケージ一覧が表示されることを確認してください．これでビルドは完了です．
    ```bash
    ros2 pkg list | grep erasers_kachaka
    ```

## Bridge イメージをビルドする
1. `erasers_kachaka` ディレクトリに移動する
    ```bash
    cd ~/colcon_ws/src/erasers_kachaka
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
    > - `KACHAKA_ERK_PATH`
    > - `ROS_DOMAIN_ID`
    > - `RMW_IMPLEMENTATION`
    > 
    > これらの環境変数は適宜変更する必要があります．

1. 次のコマンドを実施して `~/.bashrc` を再読込します．
    ```bash
    . ~/.bashrc
    ```

---

以上でローカル環境に ROS2 をインストールする手順は完了となります．

- [README にもどる](/README.md)