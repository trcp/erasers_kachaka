# ROS2 コマンドから Kachaka を制御する方法

　このチュートリアルでは ROS2 コマンドラインから Kachaka を制御する方法を解説します．<br>
　erasers_kachaka では Kachaka の様々な制御を ROS2 コマンドから呼び出せるようになっています．ここでは **ROS2 トピック通信**，**ROS2 サービス通信** それぞれの方法で Kachaka を制御する方法を解説します．

> [!IMPORTANT]
> ここでは環境変数 `KACHAKA_NAME` が `er_kachaka` となっていることを前提に解説します．

## ROS2 トピックをつかい Kachaka を制御する方法

### Kachaka から任意のテキストを発話させる方法
　Kachaka から任意のテキストを発話させるには以下のトピックを使用します．
```
/er_kachaka/kachaka_speak
```
　このトピックは
$\text{std\\_msgs/String}$
メッセージで構成されています．以下のコマンドを実行すると，Kachaka から「こんにちは」と１度だけ発話します．
```bash
ros2 topic pub --once /er_kachaka/kachaka_speak std_msgs/msg/String "data: こんにちは"
```

> [!WARNING]
> このとき，以下のコマンドのように `--once` オプション（１度だけトピックをパブリッシュするオプション）をつけ忘れると **何度も Kachaka が同じテキストを発話し続けます．**
> ```
> ros2 topic pub /er_kachaka/kachaka_speak std_msgs/msg/String "data: こんにちは"
> ```

任意のテキストを発話させたい場合はメッセージフィールド `data: ` 以降に発話させたいテキストを記述します．
```bash
ros2 topic pub --once /er_kachaka/kachaka_speak std_msgs/msg/String "data: <発話させたいテキスト>"
```

### Kachaka の音量を調整する方法
　Kachaka の音量を変更するには以下のトピックを使用します．
```
/er_kachaka/volume
```
　このトピックは
$\text{std\\_msgs/Int8}$
メッセージで構成されています．以下のコマンドを実行すると，Kachaka の音量を 10 にセットします．音量調整が成功すると Kachaka から「ポン♪」とレスポンスのビープ音がなります．
```bash
ros2 topic pub --once /er_kachaka/volume std_msgs/msg/Int8 "data: 10"
```

> [!WARNING]
> このとき，以下のコマンドのように `--once` オプション（１度だけトピックをパブリッシュするオプション）をつけ忘れると **何度も音量を変更し続けます．**
> ```
> ros2 topic pub --once /er_kachaka/volume std_msgs/msg/Int8 "data: 10"
> ```

任意の音量に調整したい場合はメッセージフィールド `data: ` 以降に整数で 0 ~ 10 の範囲の音量値を指定してトピックをパブリッシュしてください．．
```bash
ros2 topic pub --once /er_kachaka/volume std_msgs/msg/Int8 "data: <調整したい音量>"
```

> [!NOTE]
> Kachaka Pro を使用している場合音量を最大 $17$ まで設定可能です．しかし，指定可能な音量を超過すると Kachaka からレスポンスのビープ音は鳴らず，以下のワーニングが Docker コンテナログまたは bringup_launch 起動ログに表示されます．
> ```
> [WARN] [1766567246.560489900] [er_kachaka.volume_manager]: This volume value 100 is out of range.
> ```

### Kachaka のライトを調整する方法
　Kachaka の LED ライトの明るさを変更するには以下のトピックを使用します．
```
# 前方 LED
/er_kachaka/torch/front
# 後方 LED
/er_kachaka/torch/back
```
　このトピックは
$\text{std\\_msgs/Int8}$
メッセージで構成されています．以下のコマンドを実行すると，Kachaka の前方 LED の明るさを 5 にセットします．
```bash
ros2 topic pub --once /er_kachaka/torch/front std_msgs/msg/UInt8 "data: 5"
```

> [!WARNING]
> このとき，以下のコマンドのように `--once` オプション（１度だけトピックをパブリッシュするオプション）をつけ忘れると **何度も音量を変更し続けます．**
> ```
> ros2 topic pub /er_kachaka/torch/front std_msgs/msg/UInt8 "data: 5"
> ```

任意の明るさに調整したい場合はメッセージフィールド `data: ` 以降に整数で 0 ~ 255 の範囲の値を指定してトピックをパブリッシュしてください．．
```bash
ros2 topic pub --once /er_kachaka/torch/front std_msgs/msg/UInt8 "data: <調整したい明るさ>"
```

### Kachaka を速度司令で制御する方法
　Kachaka を 速度司令で制御するには以下のトピックを使用します．
```
/er_kachaka/manual_control/cmd_vel
```
　このトピックは
$\text{geometry\\_msgs/Twist}$
メッセージで構成されています．以下のコマンドを実行すると Kachaka は５秒間その場で回転します．

> [!WARNING]
> 以下のコマンドを実行すると Kachaka が移動します．

```bash
ros2 topic pub /er_kachaka/manual_control/cmd_vel geometry_msgs/msg/Twist "{angular: {z: 1.0}}" --times 5
```
以下のコマンドを実行すると Kachaka は１秒間前に進みます．
```bash
ros2 topic pub /er_kachaka/manual_control/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}}" --times 1
```

> [!HINT]
> Kachaka は負の値を与えられるとそれぞれ時計回り，または後方に移動します．


## ROS2 サービス通信をつかい Kachaka を制御する方法

### Kachaka を非常停止させる方法
　Kachaka を非常停止させるサービスは以下のとおりです．
```
/er_kachaka/emergency
````
　このトピックは
$\text{std\\_srvs/srv/Trigger}$
メッセージで構成されています．以下のコマンドを実行すると，Kachaka の LED リングが黄色になり，非常停止状態になります．
```bash
ros2 service call /er_kachaka/emergency std_srvs/srv/Trigger
```

> [!NOTE]
> このとき Kachaka の非常停止状態から復帰させるには Kachaka の電源ボタンを１回押してください．

### Kachaka のストッパーを有効にする

> [!WARNING]
> このサービスの仕様は非推奨となりました．

　Kachaka を非常停止状態ではなく通常の状態でブレーキを書けるサービスは以下のとおりです．
```
/er_kachaka/robot_stopper
```
このトピックは
$\text{std\\_srvs/srv/SetBool}$
メッセージで構成されています．以下のコマンドを実行すると，Kachaka は小刻みに前後して擬似的にブレーキがかかった状態になります．
```bash
ros2 service call /er_kachaka/robot_stopper std_srvs/srv/SetBool "data: true"
```
以下のコマンドを実行すると Kachaka のブレーキが解除されます．
```bash
ros2 service call /er_kachaka/robot_stopper std_srvs/srv/SetBool "data: false"
```

## Kachaka にシェルフを乗せる / シェルフを下ろす
　Kachaka にシェルフを載せるか，積載しているシェルフを下ろすには以下のサービスを使用します．
```
/er_kachaka/docking_shelf
```
このトピックは
$\text{std_srvs/srv/SetBool}$
メッセージで構成されています．以下のコマンドを実行すると，前方のシェルフを載せます．
```bash
ros2 service call /er_kachaka/docking_shelf std_srvs/srv/SetBool "data: true"
```
以下のコマンドを実行すると 積載しているシェルフをおろします．
```bash
ros2 service call /er_kachaka/docking_shelf std_srvs/srv/SetBool "data: false"
```
