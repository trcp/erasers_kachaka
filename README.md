# erasers_kachaka
## 依存関係のダウンロード
```bash
git clone https://github.com/GAI-313/emcl2_for_kachaka.git
git clone https://github.com/GAI-313/cartographer_ros_kachaka.git
git clone https://github.com/ros2/cartographer.git
git clone https://github.com/JMU-ROBOTICS-VIVA/ros2_aruco.git
git clone https://github.com/teamspatzenhirn/rviz_2d_overlay_plugins.git
```

## venv 作成
```bash
cd ~
sudo apt install -y python3.10-venv
python3 -m venv kachaka
```
```bash
source ~/kachaka/bin/activate
```

## kachaka-api ビルド
> 事前に `source ~/kachaka/bin/activate` を実行すること。

このリポジトリ内にダウンロードする。
```bash
# ./erasers_kachaka
git clone https://github.com/pf-robotics/kachaka-api.git
```
kachaka-api ROS2 Bridge コンテナをビルドする。
```bash
cd kachaka-api
docker buildx build -t asia-northeast1-docker.pkg.dev/kachaka-api/docker/kachaka-grpc-ros2-bridge:base --target kachaka-grpc-ros2-bridge -f Dockerfile.ros2 . --build-arg BASE_ARCH=x86_64 --load
```
カスタムファイルを kachaka-api にコピーする
```bash
# ./kachaka_api
cp ../custums/dynamic_tf_bridge.cpp ros2/kachaka_grpc_ros2_bridge/src
cp ../customs/static_tf_component.cpp ros2/kachaka_grpc_ros2_bridge/src/component
```
eR@sars 用 kachaka-api ROS2 Bridge コンテナをビルドする。
```bash
docker buildx build -t asia-northeast1-docker.pkg.dev/kachaka-api/docker/kachaka-grpc-ros2-bridge:fcsc --target kachaka-grpc-ros2-bridge -f Dockerfile.ros2 . --build-arg BASE_ARCH=x86_64 --load
```
必要な Python3 パッケージをインストールする。
```bash
# ROS2 ビルドツール
pip install catkin_pkg empy==3.3.4 lark colcon-common-extensions
# Aruco 検出用
pip install opencv-contrib-python==4.6.0.66
```
kachaka-api/python/demos に移動して kachaka_api モジュールの依存関係と本体をインストールする。
```bash
# ./kachaka_api
cd python/demos
pip install -r requirements.txt
python3 -m grpc_tools.protoc -I../../protos --python_out=. --pyi_out=. --grpc_python_out=. ../../protos/kachaka-api.proto
```
kachaka_api モジュールが使用できるかどうか確認する。<br>
なにもエラーが出なければ成功。
```bash
python3 -c "import kachaka_api" 
```

## 環境構築
　依存関係をインストールしてワークスペース直下でパッケージをビルドする。
```bash
colcon build --symlink-install --packages-up-to erasers_kachaka_bringup
```
エラーが出たら都度足りないパッケージをインストールしてください。<br>
　ビルドが完了したら以下のコマンドを実行して ~/.bashrc に登録します。
```bash
echo "source ~/YOUR_WS/install/setup.bash" >> ~/.bashrc
```
　~/.bashrc 最下層に以下のスクリプトを追加してください。`YOUR_WS` の部分はお使いの環境に合わせてください。`KACHAKA_IP` も接続するカチャカの IP アドレスにしてください。
```bash
export ROS_DOMAIN_ID=20
export ROS_LOCALHOST_ONLY=0

# kachaka
export GRPC_PORT=26400

export KACHAKA_IP=192.168.195.123

export KACHAKA_WS=~/YOUR_WS/install/setup.bash
export ER_KACHAKA_PKG=~/YOUR_WS/src/erasers_kachaka
source $KACHAKA_WS
alias kachaka_mode='source ~/kachaka/bin/activate'
alias kachaka_mode_logs='docker compose -f $ER_KACHAKA_PKG/docker-compose.yaml logs'
```
　erasers_kachaka/customs にあるカスタムアクティベーションスクリプトを kachaka venv にコピーします。
```bash
cp ~/YOUR_WS/src/erasers_kachaka/customs/activate ~/kachaka/bin/activate
```
　最後に ~/.bashrc を再読込みしたら完了です。
```bash
source ~/.bashrc
```

## 起動する
　以下のコマンドを実行してカチャカとの接続可能な状態にします。
```bash
kachaka_mode
```
kachaka_mode が起動するとバックグラウンドでカチャカとの接続が試みられます。このとき、接続先のカチャカのトーチライトが点灯したら成功です。トーチライトが点灯しない場合は接続に失敗しています。<br>
　kachaka_mode 起動中はシェルの端に `kachaka` というプロンプトが表示されます。

---

　erasers_kachaka を起動する前に以下の確認をしましょう。

- [x] kachaka_mode が起動している。（シェル左端に `kachaka` がある。）
- [x] カチャカが停止状態じゃない。（LED リングが白色である）
- [x] PS5 コントローラーと緊急停止ボタンが PC に接続されている
- [x] 十分広い場所、または安全な環境である

すべての条件が整っている場合、以下のコマンドを実行して erasers_kachaka を起動します。
```bash
ros2 launch erasers_kachaka_bringup bringup.launch.py
```
「erasers カチャカ、スタート！」とカチャカが発話したら成功です。

### 緊急停止ボタンがない。または緊急停止ボタンおよびコントローラーがない場合
　緊急停止ボタンを接続していない、または緊急停止ボタンとコントローラー両方とも接続していない場合は先程のコマンドに引数 `use_emc:=false` を追記してください。
```bash
ros2 launch erasers_kachaka_bringup bringup.launch.py use_emc:=false
```
