# マップの作成方法
　カチャカが持つローカルマップを使わず、自分自身でマップを作成する方法を解説します。

## erasers_kachaka を起動する
　以下のコマンドを実行して bringup_type を 0 にして erasers_kachaka を起動します。詳しくは
[⏩カチャカを起動する方法](/docs/howtobringup.md) 
を参照してください。
```bash
ros2 launch erasers_kachaka_bringup bringup.launch.py bringup_type:=0
```

## cartographer を起動する
　マップはデフォルトで `~/map` に作成されます。ホームディレクトリに map ディレクトリがない場合は以下のコマンドを実行して map ディレクトリを作成してください。
```bash
mkdir ~/map
```
　erasers_kachaka が起動していることを確認して以下のコマンドを実行してマップを作成します。
