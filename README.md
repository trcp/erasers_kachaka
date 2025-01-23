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
