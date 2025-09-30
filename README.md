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

## 3. 必要なパッケージをダウンロード
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

## 4, ros2_bridge コンテナのビルド
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

---

　以下のコマンドを実行して `kachaka-api` ディレクトリに移動します。
```bash
cd ../kachaka-api
```
以下のコマンドを実行してコンテナをビルドしてください。
初めてコンテナをビルドするととても長い時間がかかります。
```bash
docker buildx build -t kachaka-api:erasers --target kachaka-grpc-ros2-bridge -f Dockerfile.erk . --build-arg BASE_ARCH=x86_64 --load
```

> [!TIP]
> ネットワーク状況によってビルドにはかなりの時間がかかります。そのため上記コマンドを実行したら別のターミナルで次の手順を行うことをおすすめします。

## Python kachaka-api インストール
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

## 依存関係のインストール
　以下のコマンドを実行して必要な依存関係を自動インストールします．
```bash
cd ~/colcon_ws
sudo apt update && rosdep update
```
```bash
rosdep install -y -i --from-path src --skip-keys=ros2_aruco_interfaces --skip-keys=ros2_aruco
```

## 環境変数の設定
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

## ビルド
　`~/colcon_ws` ディレクトリに移動します。
```bash
cd ~/colcon_ws
```
　以下のコマンドを実行してワークスペース内のパッケージをビルドします。
```bash
colcon build --symlink-install --packages-up-to erasers_kachaka_bringup
```

## コード修正
- [`/colcon_ws/src/erasers_kachaka/erasers_kachaka/erasers_kachaka_navigation/launch/navigation_launch.py`](erasers_kachaka/erasers_kachaka_navigation/launch/navigation_launch.py) の変数 **`default_map`** の絶対パスを実際のコンピューターの環境に合わせてください。パスは `~/map` の **絶対パス** にしてください。`test_field.yaml` はそのままで大丈夫です。そのため以下のコマンドを実行して `~/map` を作成してください。
```bash
mkdir ~/map
```

# 起動方法
　以下のコマンドを実行して eR@sers Kachaka を起動します．
```bash
ros2 launch erasers_kachaka_bringup bringup.launch.py
```
　ロボットの起動方法に関する詳しい情報は
 [こちら](/erasers_kachaka/erasers_kachaka_bringup/README.md)
 を参照してください。
