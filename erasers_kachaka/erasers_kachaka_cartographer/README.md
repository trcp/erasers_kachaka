# erasers_kachaka_cartographer
　erasers_kachaka に SLAM 機能を提供するパッケージです。

## SLAM を行う
　SLAM を行うには事前に erasers_kachaka を bringup させる必要があります。この時、bringup されている時ナビゲーションが無効になっていないことを確認してください。
次に、以下のコマンドを実行して SLAM を行います。
```bash
ros2 launch erasers_kachaka_cartographer slam.cartographer.launch.py
```
この launch ファイルには以下のユーザーが編集可能な引数があります。

### configuration_directory
|Type|Default value|
|:---:|:---|
|str|[`erasers_kachaka_cartographer/params`](/erasers_kachaka/erasers_kachaka_cartographer/params)|

cartographer パラメータを保管している .lua ファイルを持つディレクトリまでの絶対パスを指定します。デフォルトはこのパッケージの params ディレクトリです。
```bash
ros2 launch erasers_kachaka_cartographer slam.cartographer.launch.py configuration_directory:=<lua ファイルまでの絶対パス>
```

### configuration_basename
|Type|Default value|
|:---:|:---|
|str|[`erasers_kachaka_cartographer/params/cartographer.lua`](/erasers_kachaka/erasers_kachaka_cartographer/params/cartographer.lua)|

cartographer パラメータを保管している .lua ファイルを持つディレクトリまでの絶対パスを指定します。デフォルトはこのパッケージの params ディレクトリにある cartographer.lua です。
```bash
ros2 launch erasers_kachaka_cartographer slam.cartographer.launch.py configuration_basename:=<lua ファイル名>
```

### resolution
|Type|Default value|
|:---:|:---|
|float|$`\text{0.25}`$|

cartographer により作成されるマップの解像度を設定します。デフォルトは 0.25 で、25cm 1ピクセルのマップが生成されます。さらに細かいマップを作成したい場合は小さい値を指定してください。しかし細かくしすぎると計算量が増加します。
```bash
# 生成マップ解像度を荒くする場合
ros2 launch erasers_kachaka_cartographer slam.cartographer.launch.py resolution:=0.5
```

### use_navigation
|Type|Default value|
|:---:|:---|
|bool|$`\text{true}`$|

cartographer による SLAM 中にナビゲーションを有効にする引数です。デフォルトでは Navigation が有効になっています。SLAN 中のナビゲーションを無効にする場合はこの引数に False を指定してください。
```bash
# ナビゲーションを無効にする場合
ros2 launch erasers_kachaka_cartographer slam.cartographer.launch.py use_navigation:=false
```

### auto_map_save
|Type|Default value|
|:---:|:---|
|bool|$`\text{true}`$|

cartographer による SLAM 中に生成されるマップを自動的に保存する機能を有効にする引数です。自動的にマップが保存されるのを防ぎたい場合はこの引数に False を指定してください。
```bash
# 生成マップ解像度を荒くする場合
ros2 launch erasers_kachaka_cartographer slam.cartographer.launch.py use_navigation:=false
```

### map_name
|Type|Default value|
|:---:|:---|
|str|$`\text{map}`$|

`auto_map_save` により保存されるマップの名前を指定する引数です。デフォルトでは `map` という名前で保存されます。保存されるマップの名前を変更したい場合は半角英数字で保存したいマップの名前を引数に指定してください。
```bash
# "test" という名前のマップを保存させたい場合
ros2 launch erasers_kachaka_cartographer slam.cartographer.launch.py nap_name:=test
```

### map_save_path
|Type|Default value|
|:---:|:---|
|str|[`erasers_kachaka_cartographer/map`](./map)|

`auto_map_save` により保存されるマップの保存先を指定する引数です。デフォルトではこのパッケージの map ディレクトリに保存されます。保存されるマップのパスを変更したい場合は半角英数字で保存したいマップのパスを引数に絶対パスで指定してください。<br>
デフォルト値は [slam.cartographer.launch.py](/erasers_kachaka/erasers_kachaka_cartographer/launch/slam.cartographer.launch.py) の **`SAVE_MAP_PATH`** に定義されています。**事前にコンピューターの環境にあったパス構造になっているか確認してください。**
```bash
# "test" という名前のマップをホームディレクトリに保存させたい場合
ros2 launch erasers_kachaka_cartographer slam.cartographer.launch.py map_name:=test map_save_path:=$HOME
```
