# erasers_kachaka_navigation
　Kachaka を
**`Nav2 Stack`**
を使い作成するパッケージです．

## ビルドする

> [!NOTE]
　以下の作業はビルドされていない場合，または２次開発後のみ実行します．

　手動ビルドには以下のコマンドを **ワークスペースディレクトリ内で** 実行してください．
```bash
colcon build --symlink-install --packages-up-to erasers_kachaka_navigation
```

## Navigation の起動方法

　Docker を使用している場合とローカル環境で ROS2 から直接起動させている場合で若干操作方法が異なります．

<details>
<summary>
Docker を使用している場合
</summary>

1. **erasers_kachaka を起動します．**
    ```bash
    # 内蔵マップをもたせないで起動させてください．
    docker compose up erasers_kachaka nomap_bridge
    ```

1. **Docker 起動時に表示されるターミナル「Terminator」で以下のコマンドを実行してください．**
    ```bash
    ros2 launch erasers_kachaka_navigation navigation.launch.py
    ```
    上記コマンドを実行するとデフォルトでは `~/map/test_field` マップを読み込んでナビゲーションを実行します．もし，任意のマップを読み込ませてナビゲーションを起動させたい場合は以下のコマンドを参考にしてください．以下の例では 220 という名前のマップを読み込ませています．このように手動で任意のマップを読み込ませるにはマップを作成したときに保存される YAML ファイルまでの絶対パスを `map:=` 引数に渡す必要があります．
    ```bash
    ros2 launch erasers_kachaka_navigation navigation.launch.py map:=$HOME/map/220.yaml
    ```

    |実行例|
    |:---:|
    |<img src="https://i.imgur.com/swGicb8.gif" width=70%/>|

</details>

<details>
<summary>
ローカル環境の場合
</summary>

1. **erasers_kachaka を起動します。**<br>
    　このとき、環境変数 `BRINGUP_TYPE` が `0` であることを確認してください。
    ```bash
    ros2 launch erasers_kachaka bringup.launch.py
    ```

    > 詳細は [erasers_kachaka 起動方法](/erasers_kachaka/erasers_kachaka_bringup/README.md) を参照してください。

    マップを持たずに Kachaka が起動すれば成功です。

1. **別ターミナルで以下のコマンドを実行してください．**
    ```bash
    ros2 launch erasers_kachaka_navigation navigation.launch.py
    ```
    上記コマンドを実行するとデフォルトでは `~/map/test_field` マップを読み込んでナビゲーションを実行します．もし，任意のマップを読み込ませてナビゲーションを起動させたい場合は以下のコマンドを参考にしてください．以下の例では 220 という名前のマップを読み込ませています．このように手動で任意のマップを読み込ませるにはマップを作成したときに保存される YAML ファイルまでの絶対パスを `map:=` 引数に渡す必要があります．
    ```bash
    ros2 launch erasers_kachaka_navigation navigation.launch.py map:=$HOME/map/220.yaml
    ```

    |実行例|
    |:---:|
    |<img src="https://i.imgur.com/swGicb8.gif" width=70%/>|

</details>

## Navigation の操作方法
1. **位置修正をする**<br>
    　Navigation 起動時にデフォルトで Kachaka はマップ原点に位置合わせが行われます．しかし現実の Kachaka の位置がマップ原点付近でない場合自己位置推定が失敗します．その場合次の手順で自己位置を修正します．<br>
    　以下の例では現実の Kachaka の位置が赤い矢印だとしたときのシナリオです．このように自己位置がずれている場合は RViz の「2D Pose Estimate」をクリックして Kachaka の本来の位置に矢印を置くと位置が修正されます．
    |実行例|
    |:---:|
    |<img src="https://i.imgur.com/TGBtzGb.gif" width=70%/>|


1. **2D Goal Pose をつかい kachaka を任意の場所へ自律移動させる**<br>
    　以下のように移動させたい場所に Rviz の「2D Goal Pose」マーカーを置くことで Kachaka は障害物を回避しながら目標地点への自律移動を行います．
    |実行例|
    |:---:|
    |<img src="https://i.imgur.com/9PCUMJu.gif" width=70%/>|
    |このとき Kachaka は緑色の線に沿って移動します．|
