# erasers_kachaka_teleop

　Kachaka の Teleop を実装するパッケージです．Teleop とはロボットを動かす **TeleOperation** を指します．

　このパッケージは [`erasers_kachaka_bringup`](/erasers_kachaka/erasers_kachaka_bringup) により起動を管理されています．

## ビルドする

> [!NOTE]
　以下の作業はビルドされていない場合，または２次開発後のみ実行します．

　手動ビルドには以下のコマンドを **ワークスペースディレクトリ内で** 実行してください．
```bash
colcon build --symlink-install --packages-up-to erasers_kachaka_teleop
```

## JoyStick Panel の使用方法

　Kachaka をパネルから制御する方法を解説します．`erasers_kachaka_bringup` から起動される Rviz には以下の **JoyStickPanel** が用意されています．

|**JoyStick Panel**|
|:---:|
|<img src="https://i.imgur.com/rVkfEyI.png"/>|

　上から

- 青色のプログレスバー：Kachaka の現在のバッテリー残量
- 中央のジョイスティック：Kachaka 操作用スティック
- 下部の入力欄：Kachaka に発話させたいメッセージを入力

する構成になっています．ジョイスティックを動かすか，ジョイスティックの領域でキーボード「WASD」で，Kachaka の手動操作が可能です．また，入力欄に発話させたいメッセージを書いて，エンターキーを押すと Kachaka が発話します．
なお，この入力欄は **日本語未対応です．** 日本語を発話させたい場合はローマ字入力で入力してください．

- **Kachaka を JoyStick Panel から操作する方法**
    1. erasers_kachaka を起動します．
        - Docker を使用している場合
            ```bash
            # 内蔵マップをもたせて起動させる場合
            docker compose up erasers_kachaka official_bridge
            
            # 内蔵マップをもたせないで起動させる場合
            docker compose up erasers_kachaka nomap_bridge
            ```
        - 直接起動する場合
            ```bash
            ```

    1. 起動した RViz2 の右下にある JoyStick Panel の「Enable Drive」をクリックします．すると Kachaka の手動操作が有効になります．
        |手動操作が無効な場合|手動操作が有効な場合（WASD キー制御有効）|手動操作が有効な場合（WASD キー制御無効）|
        |:---:|:---:|:---:|
        |<img src="https://i.imgur.com/rVkfEyI.png"/>|<img src="https://i.imgur.com/Mf3qmVH.png"/>|<img src="https://i.imgur.com/DV8sKvA.png"/>|

        - WASD キー制御を有効にしたい場合は「Enable Drive」を２回クリックすると再度有効になります．
        - WASD キー制御は RViz の他のパネルを操作すると無効になります．

### トピックの変更

> [!WARNING]
> 以下の紹介は erasers_kachaka パッケージ開発者向けです．下手に変更を加えると JoyStickPanel の動作に影響を及ぼします．

　このパネルでサブスクライブ，パブリッシュしているトピックの設定は JoyStickPanel 内にある **「Settings」** タブを参照してください．

<img src="https://i.imgur.com/7xFFNyq.png"/>
