# erasers_kachaka_cartographer
　Kachaka のマップを
**`cartographer`**
を使い作成するパッケージです．このパッケージは
[`erasers_kachaka_navigation`](/erasers_kachaka/erasers_kachaka_navigation)
に依存しています．

## ビルドする

> [!NOTE]
　以下の作業はビルドされていない場合，または２次開発後のみ実行します．

　手動ビルドには以下のコマンドを **ワークスペースディレクトリ内で** 実行してください．
```bash
colcon build --symlink-install --packages-up-to erasers_kachaka_cartographer
```

## マップの作成方法

　Docker を使用している場合とローカル環境で ROS2 から直接起動させている場合で若干操作方法が異なります．

<details>
<summary>
Docker を使用している場合
</summary>

1. erasers_kachaka を起動します．
    ```bash
    # 内蔵マップをもたせないで起動させてください．
    docker compose up erasers_kachaka nomap_bridge
    ```

1. Docker 起動時に表示されるターミナル「Terminator」で以下のコマンドを実行してください．
    ```bash
    ros2 launch erasers_kachaka_cartographer cartographer_launch.py
    ```
    上記コマンドを実行するとマップの作成が開始されます．

    |実行例|
    |:---:|
    |<img src="https://i.imgur.com/N4JscUk.gif" width=70%/>|

    作成されたマップはデフォルトで `test_field` という名前で `~/map` に自動的に保存されます．

    - 異なるマップ名で保存したい場合は以下のコマンドを実行します．以下のコマンドでは `test_map` という名前でマップを保存します．
        ```bash
        ros2 launch erasers_kachaka_cartographer cartographer_launch.py map_name:=test_map
        ```

    - マップを保存しないがマップを作成したい場合は以下のコマンドを実行します．
        ```bash
        ros2 launch erasers_kachaka_cartographer cartographer_launch.py use_map_save:=false
        ```

    - Navigation による自律移動を有効にするには以下のコマンドを実行します．以下の例ではマップの自動保存を無効にしつつナビゲーションを有効にします．
        ```bash
        ros2 launch erasers_kachaka_cartographer cartographer_launch.py use_map_save:=false use_navigation:=true
        ```
        　このようにナビゲーションが有効となり，「2D Goal Pose」で目標を与えるとマップを作成しながら自律移動します．

        |Navigation + SLAM|
        |:---:|
        |<img src="https://i.imgur.com/HdBkdNB.gif" width=70%/>|

1. JoyStick Panel などをつかいロボットを手動操作してマップを作成します．
    |実行例|
    |:---:|
    |<img src="https://i.imgur.com/Ci1tECo.gif" width=70%/>|

    マップ作成中は kachaka を素早く動かさないように操作してください．そうでないとマップが破綻する恐れがあります．

1. cartographer を実行している Terminator で「Control + C」を実施してマップ作成を終了します．


</details>

<details>
<summary>
ローカル環境の場合
</summary>
</details>
