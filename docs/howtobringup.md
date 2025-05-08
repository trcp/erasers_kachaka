# カチャカを起動する方法

カチャカを起動する方法を解説します。もし、
[「カチャカを接続する方法」](/docs/howtoconnect.md)
を読んでいない場合、さきに読んで準備を済ませてください。

## カチャカを起動する
　以下のコマンド（launch コマンド）を実行すると、カチャカが起動します。起動時、カチャカのライトが一時的に点灯します。
```bash
ros2 launch erasers_kachaka_bringup bringup.launch.py
```
カチャカが正常に起動しない場合、以下の事象を参考に対策してください。

- **起動時に `May be KACHAKA is not running ...` と表示される**<br>
  カチャカとの通信に失敗するとこのエラーが発生します。以下のチェックリストの順番に沿って対応してください。

  - [x] **カチャカの電源が入っている。**
    - 解決方法：カチャカの電源を入れてください。
  - [x] **カチャカとコンピューターが有線または Wi-Fi 経由で接続されている。**
    - 解決方法：カチャカとコンピューターを有線接続するか、同じ Wi-Fi 環境であることを確認してください。
  - [x] **コマンド `ping $KACHAKA_IP` でカチャカとの通信ができている。そして $KACHAKA_IP が正しいカチャカの IP アドレスに設定されている。**
    - 解決方法：[このドキュメント](/docs/howtoconnect.md) を参考にしてください。
  - [x] **[このドキュメント](/docs/erk_docker.md) を参考に、Docker コンテナが正常に動作しているか確認する。**

　デフォルトの launch コマンドではただカチャカをデフォルトの状態で起動するだけです。
ロボットの状態を可視化するツール、Rviz2 を起動したい場合、以下のように launch コマンドに引数 `use_rviz:=True` を追記してください。
```bash
ros2 launch erasers_kachaka_bringup bringup.launch.py use_rviz:=True
```

<a id="mode"></a>
## 起動モードについて
　カチャカはデフォルトでカチャカ内部のナビゲーションやマップリソースを使い起動するようになっていますが、マップを自作したい場合、またはカチャカのリソースが邪魔な場合起動モードを変更することでカチャカのリソースを使用しないで利用することができます。
launch コマンドに引数 `bringup_type:=0` を記述して起動すると、カチャカはマップとナビゲーション機能を持たずに起動します。
```bash
ros2 launch erasers_kachaka_bringup bringup.launch.py bringup_type:=0
```
　`use_rviz` を併用することでカチャカがマップを持たずに起動したことがよくわかります。
```bash
ros2 launch erasers_kachaka_bringup bringup.launch.py bringup_type:=0 use_rviz:=True
```
　bringup_type を変更するか否かの判断基準は以下を参考にしてください。

- カチャカがもつマップを使いたい場合：デフォルト
- Carry my Luggage タスクのように未知領域へのナビゲーションが必要な場合：bringup_type を 0 にする

<a id="launch"></a>
## launch ファイルについて
　以下のコマンドを実行すると launch ファイルが要求する引数一覧が表示されます。
```bash
ros2 launch erasers_kachaka_bringup bringup.launch.py --show-args
```

次のマニュアル [🎮カチャカをコントローラーで操作する方法](/docs/howtocontrol.md) を読む。
