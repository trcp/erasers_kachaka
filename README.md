# erasers_kachaka
## セットアップ方法

### 1. ワークスペースの作成
　以下のコマンドを実行してホームディレクトリに `colcon_ws` ディレクトリを作成します。
```bash
cd && mkdir -p colcon_ws/src
```

### 2. リポジトリ erasers_kachaka のダウンロード
　以下のコマンドを実行して`colcon_ws/src` ディレクトリに移動します。
```bash
cd colcon_ws/src
```
　以下のコマンドを実行して erasers_kachaka をダウンロードします。
```bash
git clone https://github.com/trcp/erasers_kachaka.git
```

### 3. 必要なパッケージをダウンロード
　以下のコマンドを実行し、`colcon_ws` ディレクトリに移動します。
```bash
cd ~/colcon_ws
```
　以下のコマンドを実行し、erasers_kachaka をビルドするために必要なパッケージを src ディレクトリにダウンロードします。
```bash
vcs import src < ~/colcon_ws/src/erasers_kachaka/setup.repos
```
　このコマンドを実行すると、以下のパッケージが src ディレクトリにダウンロードされます。

- [**kachaka-api**](https://github.com/pf-robotics/kachaka-api.git)
- [kachaka shelf description](https://github.com/GAI-313/kachaka_shelf_description.git)
- [rclpy_util](git@github.com:GAI-313/rclpy_util.git)

### 4, ros2_bridge コンテナのビルド
　以下のコマンドを実行して `erasers_kachaka` ディレクトリに移動します。
```bash
cd ~/colcon_ws/src/erasers_kachaka
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

### Python kachaka-api インストール
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
# kachaka-api インストール前に scipy をアップグレードする
pip install --break-system-packages --upgrade scipy
# kachaka-api 本体をインストール
pip install --break-system-packages --extra-index-url https://pf-robotics.github.io/kachaka-python-packages/simple kachaka-api
```
　正常にインストールが完了したら以下のコマンドを実行して正常に kachaka-api がインスt−おるされたか確認してください。
以下のコマンドを実行したとき、なにもメッセージが表示されなければ成功です。
```bash
python3 -c "import kachaka_api"
```

### 環境変数の設定
　~/.bashrc を開き、以下のコードを下に追加してください。
```bash
# kachaka
export KACHAKA_NAME="er_kachaka"
export KACHAKA_IP=192.168.195.125
export KACHAKA_ERK_PATH=~/colcon_ws/src/erasers_kachaka
export GRPC_PORT=26400
export API_GRPC_BRIDGE_SERVER_URI="${KACHAKA_IP}:${GRPC_PORT}"
```
　KACHAKA_IP は実際のカチャカのIPアドレスを指定してください。

### ビルド
　`~/colcon_ws` ディレクトリに移動します。
```bash
cd ~/colcon_ws
```
　以下のコマンドを実行してワークスペース内のパッケージをビルドします。
```bash
colcon build --symlink-install --packages-up-to erasers_kachaka_bringup
```
