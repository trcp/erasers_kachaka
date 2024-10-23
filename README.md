# erasers_kachaka
　eR@sers Education 用 Kachaka パッケージ群

## setup
　[`kachaka-api`](https://github.com/pf-robotics/kachaka-api$0)
をここにクローンしてください。
```
git clone https://github.com/pf-robotics/kachaka-api.git
```
つぎに `kachaka_bridge_main` コンテナをビルドします。
```
docker compose build kachaka_bridge_main
```
コンテナにビルドをしている間に以下のコマンドを実行して Kachaka-api 用の仮想環境を構築します。

> もし仮想環境 `venv` がインストールされていない場合はコメントされているコマンドを実行してください。

```
## if not installed venv
#sudo apt update; sudo apt install python3.10-venv -y

cd ~
python3 -m venv kachaka
source ~/kachaka/bin/activate
```
仮想環境を構築したら、このリポジトリに戻り、以下の手順で kachaka-api とその依存するパッケージをインストールします。
```
cd kachaka-api/python/demos/
pip3 install setuptools==58.2.0
pip3 install -r requirements.txt
pip3 install catkin_pkg empy lark colcon-common-extensions opencv-python
python3 -m grpc_tools.protoc -I../../protos --python_out=. --pyi_out=. --grpc_python_out=. ../../protos/kachaka-api.proto
```
### ネットワークの構築
　Kachaka を有線 LAN で使用するための手順です。
```
sudo apt -y install isc-dhcp-server
```
```
sudo nano /etc/dhcp/dhcpd.conf
```
```
sudo nano /etc/default/isc-dhcp-server
```
```
sudo systemctl restart isc-dhcp-server
sudo systemctl status isc-dhcp-server
```
### bashrc
　`~/.bashrc` を編集する前に、以下のコマンドを実行して `activate` スクリプトを変更してください。
```bash
cp customs/activate ~/kachaka/bin/activate
```
`~/.bashrc` に以下の情報を追記してください。

> `KACHAKA_WS`と`ER_KACHAKA_PKG`に適切な**絶対パス**を入力してください。

```
export GRPC_PORT=26400
export KACHAKA_IP= ...
export KACHAKA_WS= .../install/setup.bash
export ER_KACHAKA_PKG=.../src/erasers_kachaka
```
```
alias kachaka_mode='source ~/kachaka/bin/activate; source $KACHAKA_WS'
```
設定を書いたら`~/.bashrc`を読み込んでください。
```bash
source ~/.bashrc
```
### ビルドと起動
　bashrc セクションで作業した内容を行ったあと、`kachaka_mode`というコマンドが有効になります。このコマンドを実行してください。
```bash
kachaka_mode
```
　ワークスペース直下で以下のコマンドを実行して依存関係をインストールします。
```
rosdep update; rosdep install -y -i --from-path src
```
　最後に以下のコマンドを実行してパッケージをビルドします。
```
colcon build --symlink-install --packages-up-to erasers_kachaka_bringup
```
　ビルドが完了したら一度`deactivate`コマンドを実行し、もう一度`kachaka_mode`コマンドを実行し、
以下のコマンドでパッケージを起動してください。
```
ros2 launch erasers_kachaka_bringup bringup.launch.py
```
> `bringup.launch.py`の詳しい使い方は
[**コチラ**](/erasers_kachaka/erasers_kachaka_bringup/README.md)
を参照してください。

### trcp kachaka container を作成する
　以下の手順に従って erasers 用の開発向けコンテナを作成、アップデートする方法します。<br>
　kachaka-bridge の起動オプションを編集したい場合は
[`customs` ディレクトリ](/customs)を参照してください。<br>
　カスタムファイルを `kachaka-api` に反映させます。
```BASH
cd kachaka-api
cp ../customs/grpc_ros2_bridge.trcp.launch.xml ros2/kachaka_grpc_ros2_bridge/launch/
cp ../customs/grpc_ros2_bridge.trcp_devel.launch.xml ros2/kachaka_grpc_ros2_bridge/launch/
cp ../customs/dynamic_tf_bridge.cpp ros2/kachaka_grpc_ros2_bridge/src/
cp ../customs/robot_description.launch.xml ros2/kachaka_description/launch/robot_description.launch.xml
```
　次にカスタムオプションを含んだ新規コンテナを作成します。
```bash
docker buildx build -t asia-northeast1-docker.pkg.dev/kachaka-api/docker/kachaka-grpc-ros2-bridge:trcp --target kachaka-grpc-ros2-bridge -f Dockerfile.ros2 . --build-arg BASE_ARCH=x86_64 --load
```

## common api
　erasers_kachaka では `erasers_kachaka_common` パッケージが提供する **Python API** を用意しています。詳しくは
[**コチラ**](/erasers_kachaka/erasers_kachaka_common/README.md)
を参照してください。
