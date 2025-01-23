# erasers_kachaka
## セットアップ方法
### 1. kachaka-api のダウンロード
　ホームディレクトリに移動して kachaka-api をダウンロードします。
```bash
git clone https://github.com/pf-robotics/kachaka-api.git
```

### ros2_bridge コンテナのビルド
　erasers_kachaka ディレクトリに移動して kachaka-api に以下のファイルをコピーしてください。
```bash
cp docker/Dockerfile.erk ~/kachaka-api/
cp customs/grpc_ros2_bridge.trcp.launch.xml ~/kachaka-api/ros2/kachaka_grpc_ros2_bridge/launch/
```

---

　kachaka-api ディレクトリに移動して、コンテナをビルドしてください。
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

### 必要なパッケージの用意
 kachaka interfaces kachaka description

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
