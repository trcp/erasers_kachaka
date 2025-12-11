# erasers_kachaka_teleop

　Kachaka の Teleop を実装するパッケージです．Teleop とはロボットを動かす **TeleOperation** を指します．

　このパッケージは [`erasers_kachaka_bringup`](/erasers_kachaka/erasers_kachaka_bringup) により起動を管理されています．

## ビルドする

　手動ビルドには以下のコマンドを **ワークスペースディレクトリ内で** 実行してください．
```bash
colcon build --symlink-install --packages-up-to erasers_kachaka_teleop
```

## JoyStick Panel の使用方法

　Kachaka をパネルから制御する方法を解説します．`erasers_kachaka_bringup` から起動される Rviz には以下の **JoyStickPanel** が用意されています．

<img src="https://i.imgur.com/1ObPnTu.png"/>

　上から

- 青色のプログレスバー：Kachaka の現在のバッテリー残量
- 中央のジョイスティック：Kachaka 操作用スティック
- 下部の入力欄：Kachaka に発話させたいメッセージを入力

する構成になっています．ジョイスティックを動かすことで，Kachaka の手動操作が可能です．また，入力欄に発話させたいメッセージを書いて，エンターキーを押すと Kachaka が発話します．
なお，この入力欄は **日本語未対応です．** 日本語を発話させたい場合はローマ字入力で入力してください．

### トピックの変更

> [!WARNING]
> 以下の紹介は erasers_kachaka パッケージ開発者向けです．下手に変更を加えると JoyStickPanel の動作に影響を及ぼします．

　このパネルでサブスクライブ，パブリッシュしているトピックの設定は JoyStickPanel 内にある **「Settings」** タブを参照してください．

<img src="https://i.imgur.com/7xFFNyq.png"/>
