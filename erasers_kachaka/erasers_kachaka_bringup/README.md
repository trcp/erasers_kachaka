# erasers_kachaka_bringup
　erasers_kachaka を起動するパッケージです。

以下の launch ファイルを起動します。
```bash
bringup.launch.py
```
起動コマンド
```bash
ros2 launch erasers_kachaka_bringup bringup.launch.py
```
launch ファイルの引数を参照したい場合は起動コマンドに `--show-args` オプションをつけてください。
```bash
ros2 launch erasers_kachaka_bringup bringup.launch.py --show-args
```

## 引数一覧
### provide_map
|Type|Default value|
|:---:|:---|
|bool|$`\text{True}`$|

TF フレーム `map` を発行する引数です。デフォルトでは True になっており、`map` フレームを発行します。`map` フレームを発行しないようにするにはこの引数に False を代入してください。
```bash
# map フレームを発行させない
ros2 launch erasers_kachaka_bringup bringup.launch.py provide_map:=false
```

### show_rviz
|Type|Default value|
|:---:|:---|
|bool|$`\text{True}`$|

bringup 起動時に Rviz2 を表示させる引数です。デフォルトでは True になっており、Rviz2 を表示します。Rviz2 を表示しないようにするにはこの引数に False を代入してください。

> 起動する Rviz2 は [erasers_kachaka_common/config](../erasers_kachaka_common/config) に保存された rviz ファイルを使用します。

```bash
# Rviz2 を表示させない
ros2 launch erasers_kachaka_bringup bringup.launch.py show_rviz:=false
```

### use_emc
|Type|Default value|
|:---:|:---|
|bool|$`\text{True}`$|

緊急停止ボタンを使用する引数です。デフォルトでは True になっており、緊急停止ボタンがコンピューターに接続されていることを前提に bringup を移動させます。

- 緊急停止ボタンを接続していない
- 緊急停止ボタンとコントローラを接続していない

この場合はこの引数を False にしてください。

```bash
# 緊急停止ボタンを無効にする
ros2 launch erasers_kachaka_bringup bringup.launch.py use_emc:=false
```

### use_xtion
|Type|Default value|
|:---:|:---|
|bool|$`\text{False}`$|

Xtion カメラを起動させる引数です。デフォルトでは False になっており、Xtion カメラは起動しません。Xtion カメラを使用したい場合は True を代入してください。
```bash
# xtion カメラを起動する
ros2 launch erasers_kachaka_bringup bringup.launch.py use_xtion:=true
```

### use_navigation
|Type|Default value|
|:---:|:---|
|bool|$`\text{False}`$|

Navigation を起動させる引数です。デフォルトでは False になっており、Navigation は起動しません。Navigation は

- マップの読み込み
- 自己位置推定
- ナビゲーション

の3つのシステムを起動させます。Navigation を使う場合は引数に True を代入してください。しかし、マップを作成したい場合はこの引数を False にしてください。
```bash
# Navigation を起動する
ros2 launch erasers_kachaka_bringup bringup.launch.py use_navigation:=true
```

### map_name
|Type|Default value|
|:---:|:---|
|String|`2024-220-tecnofesta`|

Navigation 起動時に読み込まれるマップ名を指定する引数です。デフォルトでは `2024-220-tecnofesta` という名前のマップが読み込まれますが、[bringup.launch.py](launch/bringup.launch.py) の **`MAP`** 変数を変えることでデフォルトで読み込まれるマップ名が変化します。

```bash
# xtion カメラを起動する
ros2 launch erasers_kachaka_bringup bringup.launch.py use_navigation:=true
```

### log
|Type|Default value|
|:---:|:---|
|String|`own_log`|

起動するノードのログ設定を行う引数です。デフォルトは `own_log` です。デフォルト値ではターミナルにログは出力されません。<br>
以下のオプションを引数に与えることができ、それ以外の文字列を代入するとエラーになります。

- $`\text{screen}`$<br>
- $`\text{log}`$<br>
- $`\text{both}`$<br>
- $`\text{own\_log}`$<br>

起動ノードのログをすべて表示したい場合は引数の値を `screen` にしてください。

```bash
# 全てのログを表示させる
ros2 launch erasers_kachaka_bringup bringup.launch.py log:=screen
```
