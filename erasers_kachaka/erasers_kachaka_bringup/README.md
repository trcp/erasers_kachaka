# erasers_kachaka_bringup
　erasers_kachaka を起動するパッケージです。

## `bringup.launch.py`
　この起動ファイルを実行すると、erasers_kachaka を利用するのに必要なパッケージが起動します。以下のコマンドは最もシンプルに erasers_kachaka を起動する方法です。
```bash
ros2 launch erasers_kachaka_bringup bringup.launch.py
```
　`bringup.launch.py` は以下の **launch 引数** があります。

- **`show_rviz`**<br>
        デフォルト値 : $`\text{False}`$<br>
        **rviz** を表示するか否かを設定する引数。デフォルトの場合は rviz は表示されません。$`\text{True}`$ に設定すると起動時に rviz が起動します。<br>
        この引数を起動するには、launch 起動コマンドの末尾にオプション `show_rviz:=true` を追加します。以下が起動時に rviz を起動させるコマンドです。
        ```bash
        ros2 launch erasers_kachaka_bringup bringup.launch.py show_rviz:=true
        ```

- **`provide_map`**<br>
        デフォルト値 : $`\text{True}`$<br>
        **map_frame** を配信するか否かを設定する引数。デフォルトの場合 map_frame を配信します。これにより kachaka の TF ツリーがデフォルトで展開されます。$`\text{False}`$ に設定すると起動時に map_frame は配信されなくなり、kachaka の TF ツリーを購読することができなくなります。<br>
        この引数を起動するには、launch 起動コマンドの末尾にオプション `provide_map:=true` を追加します。以下が起動時に map_frame の配信を無効化させるコマンドです。
        ```bash
        ros2 launch erasers_kachaka_bringup bringup.launch.py provide_map:=false
        ```

### bringup.launch.py の起動時に起動される rviz の設定ファイルを指定したい場合
　この機能は今後 LaunchConfiguration に統合する予定ですが、ここでは [`bringup.launch.py`](./launch/bringup.launch.py)
を編集して設定ファイルを指定する方法を解説します。<br>
　bringup.launch.py の 28 行目の
```python
rviz_prefix = os.path.join(common_pkg_prefix, "config", rviz_name)
```
に rviz の設定ファイルパスを指定しているので、ここを編集してください。
