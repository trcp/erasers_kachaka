# erasers_kachaka_bringup
　erasers_kachaka を起動するパッケージです。以下のコマンドを実行して erasers_kachaka を起動します。
```bash
ros2 launch erasers_kachaka_bringup bringup.launch.py
```
　以下のコマンドを実行すると引数一覧が表示されます。
```bash
ros2 launch erasers_kachaka_bringup bringup.launch.py --show-args
```

## 引数一覧

- **bringup_type**<br>
  |||
  |:---:|:---:|
  |Type| Int |
  |Default| $1$ |
  
  カチャカ内蔵ナビゲーションとマップを使用するか選択する引数です。$0$ または $1$ のどちらかを選択してください。

- $0$ : かちゃか内蔵ナビゲーションとマップを使用しません。マップをもたずにカチャカは起動します。
- $1$ : カチャカ内蔵ナビゲーションとマップを使用します。

- **マップを持たずに起動させたい場合は以下のコマンドを参考にしてください。**<br>
    ```bash
    ros2 launch erasers_kachaka_bringup bringup.launch.py bringup_type:=0
    ```

- **use_rviz**<br>
  |||
  |:---:|:---:|
  |Type| Bool |
  |Default| True |
  
  起動時に RViz を起動する引数です。Rviz を表示したくない場合、またはカスタムの RViz を使用したい場合はこの引数に `False` を与えてください。
  ```bash
  ros2 launch erasers_kachaka_bringup bringup.launch.py use_rviz:=False
  ```


- **shelf_type**<br>
  |||
  |:---:|:---:|
  |Type| Int |
  |Default| $1$ |
  
  erasers_kachaka に搭載しているシェルフの種類を選択する引数です。デフォルトでは起動時に２段のシェルフがつきます。（左図）マニピュレーターを搭載したシェルフ（中央図）を使用する場合はこの引数に $2$ を与えてください。カチャカのみ（右図）使用する場合この引数に $1$ を与えてください。

  <img src="/imgs/erasers_kachaka_description.png" width=25% /><img src="/imgs/erasers_kachaka_with_manipulation_shelf_description.png" width=25% /><img src="/imgs/erasers_kachaka_without_shelf_description.png" width=25% />

- **カチャカのみ使用する場合**<br>
    ```bash
    ros2 launch erasers_kachaka_bringup bringup.launch.py shelf_type:=0
    ```
- **マニピュレーターも使用する場合**<br>
  このコマンドで起動すると搭載されているマニピュレーターも起動します。詳しくは
[erasers_kachaka_manipulation](/erasers_kachaka/erasers_kachaka_manipulation/README.md)
を参照してください。
    ```bash
    ros2 launch erasers_kachaka_bringup bringup.launch.py shelf_type:=2
    ```

## トラブルシューティング
- **起動時に `May be KACHAKA is not running ...` と表示される**<br>
  カチャカとの通信に失敗するとこのエラーが発生します。以下のチェックリストの順番に沿って対応してください。

  - [x] **カチャカの電源が入っている。**
    - 解決方法：カチャカの電源を入れてください。
  - [x] **カチャカとコンピューターが有線または Wi-Fi 経由で接続されている。**
    - 解決方法：カチャカとコンピューターを有線接続するか、同じ Wi-Fi 環境であることを確認してください。
  - [x] **コマンド `ping $KACHAKA_IP` でカチャカとの通信ができている。そして $KACHAKA_IP が正しいカチャカの IP アドレスに設定されている。**
    - 解決方法：[このドキュメント](/docs/howtoconnect.md) を参考にしてください。
  - [x] **[このドキュメント](/docs/erk_docker.md) を参考に、Docker コンテナが正常に動作しているか確認する。**

## 編集
- [bringup.launch.py](launch/bringup.launch.py) の **12行目** の `BRINGUP_MSG` ないの文字列を変更するとロボット起動時のメッセージが変わります。

# Build
```
colcon build --symlink-install --packages-select erasers_kachaka_bringup
```
or
```
colcon build --symlink-install --packages-up-to erasers_kachaka_bringup
```
