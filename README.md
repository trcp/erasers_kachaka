# erasers_kachaka

<img width=25% /><img src="/imgs/erasers_kachaka_description.png" width=50% />

<details>
<summary>
ローカル環境で eR@sers Kachaka をセットアップする方法
</summary>

# ローカル環境または WSL で eR@sers Kachaka をセットアップする方法

> **重要**<br>
> WSL で使用する推奨ディストリビューションは **Ubuntu-22.04** です。

1. **Docker をインストールする**<br>
    以下のコマンドを実行して Docker をインストールしてください。
    ```bash
    sudo apt update && sudo apt install -y ca-certificates curl gnupg lsb-release &&\
    sudo mkdir -p /etc/apt/keyrings &&\
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg &&\
    echo   "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
      $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null &&\
    sudo apt update && sudo apt install -y docker-ce docker-ce-cli containerd.io docker-compose-plugin &&\
    sudo usermod -aG docker $USER
    ```
    インストールが完了したらコンピューターを再起動してください。WSL の場合は PowerShell で以下のコマンドを実行します。
    ```
    # Windows PowerShell 上で実行します。
    wsl --shutdown
    ```
    再起動後ターミナルを開き、以下のコマンドを実行して Docker のバージョン情報が表示されたら成功です。
    ```bash
    docker -v
    ```

1. **ワークスペースを作成する**<br>
    以下のコマンドを実行してホームディレクトリに移動します。
    ```bash
    cd
    ```
    以下のコマンドを実行してワークスペースディレクトリを作成します。
    ```bash
    mkdir -p ~/colcon_ws/src
    ```

1. **erasers_kachaka をダウンロードする**<br>
    以下のコマンドを実行して ` ~/colcon_ws/src` に移動します。
    ```bash
    cd  ~/colcon_ws/src
    ```
    以下のコマンドを実行して erasers_kachaka をダウンロードします。
    ```bash
    git clone -b wsl https://github.com/trcp/erasers_kachaka.git
    ```
    
1. **Kachaka Bridge コンテナをビルドする**<br>
    erasers_kachaka ディレクトリに移動します。
    ```bash
    cd erasers_kachaka
    ```
    以下のコマンドを実行して `nomap_bridge` コンテナをビルドします。
    ```bash
    docker compose --env-file kachaka_env build nomap_bridge
    ```
    以下のコマンドを実行して `official_bridge` コンテナをビルドします。
    ```bash
    docker compose --env-file kachaka_env build official_bridge
    ```
    以下のコマンドを実行して次のような結果が得られるか確認してください。
    ```bash
    docker images
    ```
    ```
    # 実行結果
    IMAGE               ID             DISK USAGE   CONTENT SIZE   EXTRA
    gai313/ros2:kachaka_bridge.nomap
                        fde902fd84e2       2.48GB          550MB
    gai313/ros2:kachaka_bridge.official
                        5abff5c97387       2.48GB          550MB
    ```

1. **erasrs_kachaka の依存関係パッケージをダウンロードする**<br>
    以下のコマンドを実行して依存関係パッケージを自動ダウンロードします。
    ```bash
    # カレントディレクトリが erasers_kachaka であることを事前に確認してください。
    vcs import .. < ./setup.repos
    ```
    上記コマンドを実行した後、以下のコマンドを実行して `colcon_ws/src` 内に依存関係パッケージがダウンロードされていることを確認してください。
    ```
    $ ls ~/colcon_ws/src
    cartographer_ros_kachaka  erasers_kachaka  rclpy_util
    emcl2                     kachaka-api      shelf_description
    ```

1. **pip をインストールする**<br>
    以下のコマンドを実行して Python 管理パッケージ `pip` をインストールします。
    ```bash
    sudo apt install -y python3-pip
    ```
    次に以下のコマンドを実行してインストールした pip を最新バージョンに更新します。
    ```bash
    python3 -m pip install --upgrade pip
    ```
    
1. **Python 依存関係パッケージをインストールする**<br>
    以下のコマンドを実行して Python 依存パッケージをインストールします。
    ```bash
    # カレントディレクトリが erasers_kachaka であることを事前に確認してください。
    pip3 install -r requirements.txt
    ```
    以下のコマンドを実行して `kachaka-api` が利用可能か確認します。実行結果に何も応答がなければ成功です。
    ```bash
    python3 -c "import kachaka_api"
    ```

1. **ROS2 依存関係を自動解決する**<br>
    `~/colcon_ws` ディレクトリに移動します。
    ```bash
    cd ~/colcon_ws
    ```
    以下のコマンドを実行して apt と rosdep を更新します。
    ```bash
    sudo apt update && rosdep update
    ```
    以下のコマンドを実行して ROS2 依存関係パッケージを自動インストールします。
    ```bash
    rosdep install -y -i --from-path src --skip-keys=ros2_aruco_interfaces --skip-keys=ros2_aruco
    ```

1. **ワークスペースをビルドする**<br>
    以下のコマンドを実行してワークスペースをビルドします。
    ```bash
    # カレントディレクトリが colcon_ws  であることを事前に確認してください。
    colcon build --symlink-install --packages-up-to erasers_kachaka_bringup
    ```

1. **`~/.bashrc` に環境変数を追記する**<br>
    ~/.bashrc に以下の環境変数を追記してください。
    ```bash
    # kachaka
    source ~/colcon_ws/install/setup.bash
    export KACHAKA_NAME="er_kachaka"
    export KACHAKA_IP=192.168.195.125
    export USE_TOF_POINTS=True
    export USE_RVIZ=True
    export BRINGUP_TYPE=0
    export SHELF_TYPE=0
    export KACHAKA_ERK_PATH=~/colcon_ws/src/erasers_kachaka
    ## DO NOT EDIT !!!!
    export GRPC_PORT=26400 
    export API_GRPC_BRIDGE_SERVER_URI="${KACHAKA_IP}:${GRPC_PORT}"

    # ROS
    export ROS_DOMAIN_ID=0
    export ROS_LOCALHOST_ONLY=0
    export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
    ```
    書き込んだらいかのコマンドを実行して ~/.bashrc を再読み込みします。
    ```bash
    source ~/.bashrc
    ```
    以下のコマンドを実行して環境変数の内容が launcher の引数に対応しているかどうか確認してください。
    ```bash
    ros2 launch erasers_kachaka_bringup bringup.launch.py --show-args
    ```
    初期設定では以下のように表示されるはずです。
    ```
    Arguments (pass arguments as '<name>:=<value>'):

    'namespace':
        Robot Namespace
        (default: 'er_kachaka')

    'robot_ip':
        Robot IP address
        (default: '192.168.195.125')

    'bringup_docker':
        Launch docker container automatic.
        (default: 'True')

    'bringup_type':
        Select bringup docker container type: [0, 1]. Please read doc about detail.
        (default: '0')

    'shelf_type':
        Select shelf model type:[0, 1, 2]/ Please read doc about detail.
        (default: '0')

    'publish_tof_pc2':
        Enable publish TOF Pointcloud2 topic from Kachaka front sensor.
        (default: 'True')

    'use_rviz':
        Launch Rviz2
        (default: 'True')

    'use_shelf':
        Docking shelf
        (default: 'false')

    'use_sim_time':
        simulation time
        (default: 'false')
    ```
    
</details>

<details>
<summary>
Docker から eR@sers Kachaka をセットアップする
</summary>

# Docker から eR@sers Kachaka をセットアップする
## erasers_kachaka イメージをビルドする
　以下のコマンドを実行してコンテナ内で使用する任意のパスワードを用意してください．
```bash
export PASSWORD=<password>
```
　以下のコマンドを実行して eR@sers Kachaka コンテナをビルドします．
```bash
docker compose build --env-file kachaka_env erasers_kachaka
```

> 
> erasers_kachaka コンテナをビルドしているときに以下のエラーが発生した場合，環境変数 `PASSWORD` が未定義であるか，変数内が空である可能性があります．もう一度この環境変数に任意のパスワードを定義して再実行してください．
> ```
> chpasswd: (line 1, user USERNAME) password not changed
> ```

## Kachaka と接続する設定を行う
　ビルドが完了したらリポジトリ直下にある [kachaka_env](/kachaka_env) を開き，変数 `KACHAKA_IP` を接続したい Kachaka の IP アドレスに設定してください．
```diff
...
# IP Address for Kachaka
- KACHAKA_IP=192.168.195.121
+ KACHAKA_IP=XXX.XXX.XXX.XXX
...
```

> 
> Wi-Fi 経由で Kachaka と接続する場合，Kachaka に「ねぇカチャカ，IP アドレスを教えて」と尋ねると IP アドレスを教えてくれます．

他にも kachaka_env ファイルには erasers_kachaka を利用するための環境変数をが用意されています．それぞれの値は以下の表を参照してください．

|変数名|意味|
|:---|:---|
|**UID**|コンテナ内のユーザー権限設定．編集しないでください．|
|**USER_ID**|コンテナ内のユーザー権限設定．編集しないでください．|
|**GID**|コンテナ内のグループ権限設定．編集しないでください．|
|**GROUP_ID**|コンテナ内のグループ権限設定．編集しないでください．|
|**ROS_DOMAIN_ID**|コンテ内で利用する ROS_DOMAIN_ID を設定します．他のロボットからの干渉を防ぐには任意の値に設定することを推奨します．|
|**ROS_LOCALHOST_ONLY**|LAN 上にトピックを公開したくない場合はこの変数に `1` を代入してください．|
|**RMW_IMPLEMENTATION**|ROS2 通信に使用する DDS プロトコルを設定します．`rmw_fastrtps_cpp` と `rmw_cyclonedds_cpp` を選択できます．|
|**KACHAKA_NAME**|ロボットの名前空間を定義します．`er_kachaka` の場合，`/er_kachaka/...` のトピックらを取得，出力します．|
|**KACHAKA_IP**|Kachaka の IP アドレスを定義します．|
|**USE_RVIZ**|erasers_kachaka コンテナ起動時に RViz を表示，非表示にします．|
|**USE_TOF_POINTS**|Kachaka の前方 ToF カメラから PointCloud2 をパブリッシュします．|
|**BRINGUP_TYPE**|この変数は使われていません．|
|**SHELF_TYPE**|0, 1, 2 のいづれかを定義します．起動時に使われる Robot Description の種類を選択します．<br>０：Kachaka のみ<br><img src="https://i.imgur.com/3QpGqCA.png" /><br>１：シェルフを積載した Kachaka<br><img src="https://i.imgur.com/LSJ5DwV.png" /><br>２：なにもなし<br><img src="https://i.imgur.com/3HtXu9S.png" />|
|**GRPC_PORT**|Kachaka との通信に必要な変数です．編集しないでください．|
|**API_GRPC_BRIDGE_SERVER_URI**|Kachaka との通信に必要な変数です．編集しないでください．|

## erasers_kachaka を起動する
　使用するコンピュータが起動して初めて利用する場合は以下のコマンドを実行して Docker が GUI を出力できるようにしてください．
```bash
xhost +
```
　起動方法には２つのオプションがあります．

- **Kachaka App で作成したマップを使用する場合**<br>
    Kachaka の内蔵マップを使いたい場合はこちらを実行してください．
    ```bash
    docker compose --env-file kachaka_env up erasers_kachaka official_bridge
    ```
    <img src="https://i.imgur.com/B7ThilZ.png"/>
- **マップを持たずに使用する場合**<br>
    自作マップを利用したい場合はこちらを実行してください．
    ```bash
    docker compose --env-file kachaka_env up erasers_kachaka nomap_bridge
    ```
    <img src="https://i.imgur.com/IlfDoiT.png"/>

　コンテナ起動時に以下のようなターミナル「Terminator」が起動します．erasers_kachaka コンテナ内で ROS2 などの操作をする場合はこちらで行います．
<br><img srcc="https://i.imgur.com/ebz08kS.png"/>

</details>

---

　Kachaka との接続に成功すると Kachaka から「Kachaka スタート！」と発話します．そして表示される Rviz にロボットからのステータスが表示されます．
<br><img src="https://i.imgur.com/We4FrEm.jpeg" />

　Rviz に表示されるデフォルトの情報は次のとおりです．

||||
|:---:|:---:|:---|
|**Front Camera**|<img src="https://i.imgur.com/OMsDhef.png"/>|　Kachaka 前方カメラからの映像を表示します．また前方カメラから見た検出物体の推定位置も描画されますが，カメラ画像に直接描画されているものではありません．|
|**Object Detect Image**|<img src="https://i.imgur.com/YQJqMSO.png"/>|　Kachaka 前方カメラから検出した物体情報を可視化した情報を表示します．「 *Not Detected Objects* 」と表示されている場合，Kachaka は物体を検出できていないことを示しています．|
|**Back Camera**|<img src="https://i.imgur.com/tQFrurW.png"/>|　Kachaka 後方カメラビューを表示します．|
|**JoyStick Panel**|<img src="https://i.imgur.com/rVkfEyI.png"/>|　Kachaka をジョイスティックで移動したり，バッテリー残量の確認，発話テキストの送信ができるパネルです．|
|**Kachaka**|<img src="https://i.imgur.com/TQKgWtz.png"/>|　Kachaka のロボットモデル（Robot Description）を表示します．Docker から起動するとき，[kachaka_env](kachaka_env) の `SHELF_TYPE` によってロボットの見た目が変わります．|
|**LiDAR**|<img src="https://i.imgur.com/qymZagf.png"/>|　Kachaka の LiDAR センサーから検出した障害物を紫色のパーティクルで示します．|
|**Map**|<img src="https://i.imgur.com/NTJoPX0.png"/>|　Kachaka から作成されたマップを描画します．Cartographer, Navigation などを起動すると表示されます．|
|**LocalCostMap**|<img src="https://i.imgur.com/ZRfBbVL.png"/>|　ナビゲーション時のロボット周囲の障害物に対するコストマップを描画します（明るい水色と赤のパーティクル）．|
|**GlobalCostMap**|<img src="https://i.imgur.com/ZRfBbVL.png"/>|　ナビゲーション時のマップと各障害物に対するコストマップを描画します（淡い水色と赤のパーティクル）．|
|**TF**|<img src="https://i.imgur.com/8ulIjro.png"/>|　ロボットの現在の座標系を X（赤軸）,Y（緑軸）,Z軸（青軸）で示します．|
|**Path**|<img src="https://i.imgur.com/ZRfBbVL.png"/>|　ナビゲーション時のロボットの起動を描画します（緑色の線）．|
|**Kachaka Detect Object Pose**|<img src="https://i.imgur.com/05aIscc.png"/>|　検出した物体の推定位置を X,Y,Z 軸で描画します．TF と酷似していますがこの軸のほうが太いです．|
|**Kachaka Detect Object Marker**|<img src="https://i.imgur.com/05aIscc.png"/>|　検出した物体の推定サイズと位置を描画します．描画されたボックスが検出した物体の推定サイズを示し，ボックス内中央に検出した物体名が描画されます．|
|**Goal Pose**|<img src="https://i.imgur.com/ZRfBbVL.png"/>|　ナビゲーション時のロボットの到達位置を描画します（赤い矢印）．|

---

# Tutorials
- [erasers_kachaka 起動方法と設定方法](/erasers_kachaka/erasers_kachaka_bringup/README.md)
- [Kachaka を JoyStick Panel から操作する](/erasers_kachaka/erasers_kachaka_teleop/README.md)
- [cartographer でマップを作成する方法](/erasers_kachaka/erasers_kachaka_cartographer/README.md)
- [Kachaka を Navigation で自律移動させる方法](/erasers_kachaka/erasers_kachaka_navigation/README.md)
- [ROS2 コマンドラインから Kachaka を制御する方法](/docs/ros2_command.md)
