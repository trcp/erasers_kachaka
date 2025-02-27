# erasers_kachaka_bringup
　erasers_kachaka を起動するパッケージです。以下のコマンドを実行して erasers_kachaka を起動します。
```bash
ros2 launch erasers_kachaka_bringup bringup.launch.py
```

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
