> [!WARNING]
> 個人開発用ブランチです．将来的に main にマージ予定ですが，このブランチをフォークすることは推奨しません．

# erasers_kachaka

<img width=25% /><img src="/imgs/erasers_kachaka_description.png" width=50% />

[English](README_en.md) | 日本語

- [ローカル環境に eR@sers Kachaka をインストールする方法](/docs/install_local..md)
- [Docker 環境に eR@sers Kachaka をインストールする方法](/docs/install_on_docker.md)

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

---

# Tutorials
- [erasers_kachaka 起動方法と設定方法](/erasers_kachaka/erasers_kachaka_bringup/README.md)
- [Kachaka を JoyStick Panel から操作する](/erasers_kachaka/erasers_kachaka_teleop/README.md)
- [cartographer でマップを作成する方法](/erasers_kachaka/erasers_kachaka_cartographer/README.md)
- [Kachaka を Navigation で自律移動させる方法](/erasers_kachaka/erasers_kachaka_navigation/README.md)
- [ROS2 コマンドラインから Kachaka を制御する方法](/docs/ros2_command.md)
