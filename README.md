# erasers_kachaka

<img width=25% /><img src="/imgs/erasers_kachaka_description.png" width=50% />

<!-- 
# 開発に関する手引きとマニュアル、トラブルシューティングドキュメント一覧
## 📝マニュアル
- [⏩カチャカと接続する方法](/docs/howtoconnect.md)
  - [🔌有線接続する方法](/docs/howtoconnect.md#ethernet)
  - [🛜無線接続する方法](/docs/howtoconnect.md#wireless)
- [⏩カチャカを起動する方法](/docs/howtobringup.md)
  - [🕹️起動モードについて](/docs/howtobringup.md#mode)
  - [🚀Launchファイルについて](/docs/howtobringup.md#launch) 
- [🎮カチャカをコントローラーで操作する方法](/docs/howtocontrol.md)
- [🔈カチャカから発話させる方法](/docs/howtospeak.md)
- [🗺マップの作成方法](/docs/howtomap.md)

## ⚒開発関連
- [🐱開発のはじめ方](/docs/develop.md)
- [🐳ros2_bridge kachaka Docker コンテナの起動チェック](/docs/erk_docker.md)

## 🗒チュートリアル
- [🚗カチャカを移動させる方法](/docs/howtomove.md)
- [🗺マップの作成方法](/docs/howtomap.md)
- [💫ナビゲーション方法](/docs/howtonav.md)

-->

# セットアップ方法

　前提として Docker をインストールしてください．

## 1. ワークスペースの作成
　以下のコマンドを実行してホームディレクトリに `colcon_ws` ディレクトリを作成します。
```bash
cd && mkdir -p colcon_ws/src
```

## 2. リポジトリ erasers_kachaka のダウンロード
　以下のコマンドを実行して`colcon_ws/src` ディレクトリに移動します。
```bash
cd colcon_ws/src
```
　以下のコマンドを実行して erasers_kachaka をダウンロードします。
```bash
git clone https://github.com/trcp/erasers_kachaka.git
```

## 3, ros2_bridge コンテナのビルド
<!-- 
　以下のコマンドを実行して `erasers_kachaka` ディレクトリに移動します。
```bash
cd ./erasers_kachaka
```
　以下のコマンドを実行して kachaka-api に必要なファイルをコピーします。
```bash
cp docker/Dockerfile.erk ../kachaka-api/
cp customs/grpc_ros2_bridge.trcp.launch.xml ../kachaka-api/ros2/kachaka_grpc_ros2_bridge/launch/
cp customs/dynamic_tf_bridge.cpp ~/colcon_ws/src/kachaka-api/ros2/kachaka_grpc_ros2_bridge/src/dynamic_tf_bridge.cpp
cp customs/static_tf_component.cpp ~/colcon_ws/src/kachaka-api/ros2/kachaka_grpc_ros2_bridge/src/component/static_tf_component.cpp
```
-->

　以下のコマンドを実行して Kachaka と通信するための Docker コンテナをビルドします．
```bash
docker compose build nomap_bridge official_bridge 
```

> [!TIP]
> ネットワーク状況によってビルドにはかなりの時間がかかります。そのため上記コマンドを実行したら別のターミナルで次の手順を行うことをおすすめします。

## 4. 必要なパッケージをダウンロード

> [!NOTE]
> ここから先，手順 9 までの内容は **ローカル環境に ROS2 Humble がインストールされている** ことを前提に解説しています．<br>
> Docker 環境を使い eR@sers Kachaka を利用したい場合は **Docker から eR@sers Kachaka をセットアップする** を参照してください．

　以下のコマンドを実行し、erasers_kachaka をビルドするために必要なパッケージを src ディレクトリにダウンロードします。
```bash
vcs import . < ./erasers_kachaka/setup.repos
```
　このコマンドを実行すると、以下のパッケージが src ディレクトリにダウンロードされます。

- [**kachaka-api**](https://github.com/pf-robotics/kachaka-api.git)
- [kachaka shelf description](https://github.com/GAI-313/kachaka_shelf_description.git)
- [rclpy_util](https://github.com/GAI-313/rclpy_util.git)
- [cartographer](https://github.com/ros2/cartographer.git)
- [cartographer_ros_kachaka](https://github.com/GAI-313/cartographer_ros_kachaka.git)
- [emcl2](https://github.com/GAI-313/emcl2_for_kachaka.git)


> OPL 使用にしたい場合は続けて以下のコマンドを実行してください。
> ```bash
> vcs import . < ./erasers_kachaka/opl.repos
> ```

## 5. Python kachaka-api インストール
　pip3 がインストールされていない場合は以下のコマンドを実行して pip3 をインストールしてください。
```bash
sudo apt install -y python3-pip
```
　次に以下のコマンドを実行して pip を更新してください。
```bash
python3 -m pip install --upgrade pip
```
　実環境上にインストールする場合、以下のコマンドを実行して kachaka-api をインストールしてください。
```bash
pip install kachaka-api
pip install "scipy>=1.13.0" transform3d matplotlib numpy==1.22.4
```
　正常にインストールが完了したら以下のコマンドを実行して正常に kachaka-api がインスt−おるされたか確認してください。
以下のコマンドを実行したとき、なにもメッセージが表示されなければ成功です。
```bash
python3 -c "import kachaka_api"
```

## 6. 依存関係のインストール
　以下のコマンドを実行して必要な依存関係を自動インストールします．
```bash
cd ~/colcon_ws
sudo apt update && rosdep update
```
```bash
rosdep install -y -i --from-path src --skip-keys=ros2_aruco_interfaces --skip-keys=ros2_aruco
```


## 7. ビルド
　`~/colcon_ws` ディレクトリに移動します。
```bash
cd ~/colcon_ws
```
　以下のコマンドを実行してワークスペース内のパッケージをビルドします。
```bash
colcon build --symlink-install --packages-up-to erasers_kachaka_bringup
```

## 8. 環境変数の設定
　~/.bashrc を開き、以下のコードを下に追加してください。
```bash
# kachaka
export KACHAKA_NAME="er_kachaka"
export KACHAKA_IP=192.168.195.125
export KACHAKA_ERK_PATH=~/colcon_ws/src/erasers_kachaka
export GRPC_PORT=26400
export API_GRPC_BRIDGE_SERVER_URI="${KACHAKA_IP}:${GRPC_PORT}"

# ROS
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
　KACHAKA_IP は実際のカチャカのIPアドレスを指定してください。

> [!IMPORTANT]
> - `ROS_DOMAIN_ID` は状況に応じて任意の番号にしてください．
> - `GRPC_PORT=26400` は Kachaka と通信するために必要な変数です．値は変更しないでください．
> - `ROS_LOCALHOST_ONLY=0` は Kachaka と通信するために必要な変数です．値は変更しないでください．
> - `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` は状況に応じて任意の DDS を使用してください．

# 起動方法
　以下のコマンドを実行して eR@sers Kachaka を起動します．
```bash
ros2 launch erasers_kachaka_bringup bringup.launch.py
```
　ロボットの起動方法に関する詳しい情報は
 [こちら](/erasers_kachaka/erasers_kachaka_bringup/README.md)
 を参照してください。

# Docker から eR@sers Kachaka をセットアップする
## erasers_kachaka イメージをビルドする
　以下のコマンドを実行してコンテナ内で使用する任意のパスワードを用意してください．
```bash
export PASSWORD=<password>
```
　以下のコマンドを実行して eR@sers Kachaka コンテナをビルドします．
```bash
docker compose build erasers_kachaka
```

> [!WARNING]
> erasers_kachaka コンテナをビルドしているときに以下のエラーが発生した場合，環境変数 `PASSWORD` が未定義であるか，変数内が空である可能性があります．もう一度この環境変数に任意のパスワードを定義して再実行してください．
> ```
> chpasswd: (line 1, user USERNAME) password not changed
> ```

## Kachaka と接続する設定を行う
　ビルドが完了したらリポジトリ直下にある [.env](/.env) を開き，変数 `KACHAKA_IP` を接続したい Kachaka の IP アドレスに設定してください．
```diff
...
# IP Address for Kachaka
- KACHAKA_IP=192.168.195.121
+ KACHAKA_IP=XXX.XXX.XXX.XXX
...
```

> [!TIP]
> Wi-Fi 経由で Kachaka と接続する場合，Kachaka に「ねぇカチャカ，IP アドレスを教えて」と尋ねると IP アドレスを教えてくれます．

他にも .env ファイルには erasers_kachaka を利用するための環境変数をが用意されています．それぞれの値は以下の表を参照してください．

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
    docker compose up erasers_kachaka official_bridge
    ```
    <img src="https://i.imgur.com/B7ThilZ.png"/>
- **マップを持たずに使用する場合**<br>
    自作マップを利用したい場合はこちらを実行してください．
    ```bash
    docker compose up erasers_kachaka nomap_bridge
    ```
    <img src="https://i.imgur.com/IlfDoiT.png"/>

コンテナ起動時に以下のようなターミナル「Terminator」が起動します．erasers_kachaka コンテナ内で ROS2 などの操作をする場合はこちらで行います．
<br><img src="https://i.imgur.com/ebz08kS.png"/>
