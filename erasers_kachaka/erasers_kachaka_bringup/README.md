# erasers_kachaka_bringup

　erasers_kachaka の起動を司るパッケージです．

## ビルドする

> [!NOTE]
　以下の作業はビルドされていない場合，または２次開発後のみ実行します．

　手動ビルドには以下のコマンドを **ワークスペースディレクトリ内で** 実行してください．
```bash
colcon build --symlink-install --packages-up-to erasers_kachaka_bringup
```

## erasers_kachaka 起動方法
　Docker を使用している場合とローカル環境で ROS2 から直接起動させている場合で若干操作方法が異なります．

<details>
<summary>
Docker を使用している場合
</summary>

1. **erasers_kachaka 内にある [kachaka_env](/kachaka_env) を編集します．**<br>
    このファイルの `KACHAKA_IP` に記述されている IP アドレスを接続したい kachaka の IP アドレスに変更してください．
    ```
    KACHAKA_IP=<接続先の IP アドレス>
    ```

1. **erasers_kachaka コンテナと Bridge コンテナを起動する**<br>
    以下のコマンドを実行して erasers_kachaka と Bridge コンテナを起動します．
    ```
    # 推奨
    # Kachaka 内蔵マップなしで起動したい場合
    docker compose --env-file kachaka_env up erasers_kachaka nomap_bridge
    ```
    ```
    # Kachaka 内蔵マップ有りで起動したい場合
    docker compose --env-file kachaka_env up erasers_kachaka official_bridge
    ```
</details>

<details>
<summary>
ローカル環境の場合
</summary>

1. **`KACHAKA_IP` を確認します。**<br>
    　以下のコマンドを実行して設定されている `KACHAKA_IP` の IP アドレスで Kachaka と通信可能か確認してください。
    ```bash
    ping $KACHAKA_IP
    ```

    > 上記コマンドは「Control + C」で停止できます。

    　もし、上記コマンドを実行して何もレスポンスがない場合は以下の作業を行い Kachaka と接続できるようにしてください。

    1. **`~/.bashrc` を編集する**<br>
        　`~/.bashrc` 内で設定されている環境変数 `KACHAKA_IP` の IP アドレス値を接続したい Kachaka の IP アドレスに書き換えてください。
        ```diff
        - export KACHAKA_IP=192.168.195.125
        + export KACHAKA_IP=<接続先の IP アドレス>
        ```

        > Wi-Fi 経由で接続したい場合、接続先の Kachaka の IP アドレスは Kachaka に「ねぇカチャカ、IP アドレスを教えて。」とリクエストすることで確認できます。

    1. **`~/.bashrc` を再読み込みする**<br>
        ```
        source ~/.bashrc
        ```

    1. **再度接続確認する**<br>
        ```bash
        ping $KACHAKA_IP
        ```

1. **erasers_kachaka の起動設定を確認する**<br>
    　`~/.bashrc` 内に記述した各環境変数はそれぞれ次の意味を持ちます。

    |変数名|意味|
    |:---|:---|
    |**ROS_DOMAIN_ID**|コンテ内で利用する ROS_DOMAIN_ID を設定します．他のロボットからの干渉を防ぐには任意の値に設定することを推奨します．|
    |**ROS_LOCALHOST_ONLY**|LAN 上にトピックを公開したくない場合はこの変数に `1` を代入してください．|
    |**RMW_IMPLEMENTATION**|ROS2 通信に使用する DDS プロトコルを設定します．`rmw_fastrtps_cpp` と `rmw_cyclonedds_cpp` を選択できます．|
    |**KACHAKA_NAME**|ロボットの名前空間を定義します．`er_kachaka` の場合，`/er_kachaka/...` のトピックらを取得，出力します．|
    |**KACHAKA_IP**|Kachaka の IP アドレスを定義します．|
    |**USE_RVIZ**|erasers_kachaka コンテナ起動時に RViz を表示，非表示にします．|
    |**USE_TOF_POINTS**|Kachaka の前方 ToF カメラから PointCloud2 をパブリッシュします．|
    |**BRINGUP_TYPE**|０，１のいづれかを定義します。<br>０：マップを持たずに起動する。<br>１：Kachaka 内臓マップを持たせて起動する。|
    |**SHELF_TYPE**|0, 1, 2 のいづれかを定義します．起動時に使われる Robot Description の種類を選択します．<br>０：Kachaka のみ<br><img src="https://i.imgur.com/3QpGqCA.png" /><br>１：シェルフを積載した Kachaka<br><img src="https://i.imgur.com/LSJ5DwV.png" /><br>２：なにもなし<br><img src="https://i.imgur.com/3HtXu9S.png" />|
    |**GRPC_PORT**|Kachaka との通信に必要な変数です．編集しないでください．|
    |**API_GRPC_BRIDGE_SERVER_URI**|Kachaka との通信に必要な変数です．編集しないでください．|

    `~/.bashrc` を編集するか、`export` コマンドで環境変数を上書きして起動方法を設定してください。

    > `~/.bashrc` を編集した場合は以下のコマンドを実行して変更内容を反映させてください。
    > ```bash
    > source ~/.bashrc
    > ```

1. **erasers_kachaka を起動する**<br>
    ```bash
    ros2 launch erasers_kachaka_bringup bringup.launch.py
    ```

    > Kachaka から「Kachaka、スタート。」と発話すれば起動成功です。

</details>

---

　Kachaka との接続に成功すると Kachaka から「Kachaka スタート！」と発話します．そして表示される Rviz にロボットからのステータスが表示されます．
<br><img src="https://i.imgur.com/We4FrEm.jpeg" />

　Rviz に表示されるデフォルトの情報は次のとおりです．

||||
|:---:|:---:|:---|
|**Front Camera**|<img src="https://i.imgur.com/OMsDhef.png"/>|　Kachaka 前方カメラからの映像を表示します．また前方カメラから見た検出物体の推定位置も描画されますが，カメラ画像に直接描画されているものではありません．|
|**Object Detect Image**|<img src="https://i.imgur.com/YQJqMSO.png"/>|　Kachaka 前方カメラから検出した物体情報を可視化した情報を表示します．「 *Not Detected Objects* 」と表示されている場合，Kachaka は物体を検出できていないことを示しています．|
|**Back Camera**|<img src="https://i.imgur.com/tQFrurW.png"/>|　Kachaka 後方カメラビューを表示します．|
|**JoyStick Panel**|<img src="https://i.imgur.com/rVkfEyI.png"/>|　Kachaka をジョイスティックで移動したり，バッテリー残量の確認，発話テキストの送信ができるパネルです．|
|**Kachaka**|<img src="https://i.imgur.com/TQKgWtz.png"/>|　Kachaka のロボットモデル（Robot Description）を表示します．Docker から起動するとき，[kachaka_env](kachaka_env) の `SHELF_TYPE` によってロボットの見た目が変わります．|
|**LiDAR**|<img src="https://i.imgur.com/qymZagf.png"/>|　Kachaka の LiDAR センサーから検出した障害物を紫色のパーティクルで示します．|
|**Map**|<img src="https://i.imgur.com/NTJoPX0.png"/>|　Kachaka から作成されたマップを描画します．Cartographer, Navigation などを起動すると表示されます．|
|**LocalCostMap**|<img src="https://i.imgur.com/ZRfBbVL.png"/>|　ナビゲーション時のロボット周囲の障害物に対するコストマップを描画します（明るい水色と赤のパーティクル）．|
|**GlobalCostMap**|<img src="https://i.imgur.com/ZRfBbVL.png"/>|　ナビゲーション時のマップと各障害物に対するコストマップを描画します（淡い水色と赤のパーティクル）．|
|**TF**|<img src="https://i.imgur.com/8ulIjro.png"/>|　ロボットの現在の座標系を X（赤軸）,Y（緑軸）,Z軸（青軸）で示します．|
|**Path**|<img src="https://i.imgur.com/ZRfBbVL.png"/>|　ナビゲーション時のロボットの起動を描画します（緑色の線）．|
|**Kachaka Detect Object Pose**|<img src="https://i.imgur.com/05aIscc.png"/>|　検出した物体の推定位置を X,Y,Z 軸で描画します．TF と酷似していますがこの軸のほうが太いです．|
|**Kachaka Detect Object Marker**|<img src="https://i.imgur.com/05aIscc.png"/>|　検出した物体の推定サイズと位置を描画します．描画されたボックスが検出した物体の推定サイズを示し，ボックス内中央に検出した物体名が描画されます．|
|**Goal Pose**|<img src="https://i.imgur.com/ZRfBbVL.png"/>|　ナビゲーション時のロボットの到達位置を描画します（赤い矢印）．|
