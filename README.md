## setup
　[`kachaka-api`]()
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
#udo apt update; sudo apt install python3.10-venv -y

cd ~
python3 -m venv kachaka
source ~/kachaka/bin/activate
```
仮想環境を構築したら、このリポジトリに戻り、以下の手順で kachaka-api とその依存するパッケージをインストールします。
```
cd kachaka-api/python/demos/
pip3 install -r requirements.txt
pip3 install catkin_pkg empy lark colcon-common-extensions
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

### bashrc
```
`~/.bashrc` に以下の情報を追記してください。
```
export GRPC_PORT=26400
export KACHAKA_IP= ...
export KACHAKA_WS= .../install/setup.bash
```
```
alias kachaka_mode='source ~/kachaka/bin/activate; source $KACHAKA_WS'
```

### ビルドと起動
　このリポジトリに移動して、以下のコマンドを実行して kachaka-ros-bridge を起動します。
```
docker compose up kachaka_bridge_main
```
　ワークスペース直下で以下のコマンドを実行して依存関係をインストールします。
```
rosdep update; rosdep install -y -i --from-path src
```
　最後に以下のコマンドを実行してパッケージをビルドします。
```
colcon build --symlink-install --packages-up-to erasers_kachaka_bringup
```
　ビルドが完了したら `kachaka-mode` を起動して以下のコマンドを実行して起動します
```
ros2 launch erasers_kachaka_bringup bringup.launch.py
```
