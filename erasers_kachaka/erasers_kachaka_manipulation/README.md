# erasers_kachaka_manipulation
　erasers_kachaka に搭載するマニピュレーター関連のパッケージです。以下のコマンドを実行して erasers_manipulator を起動します。以下のコマンドは **erasers_kachaka_bringup にて引数 `shelf_type` に $2$ を渡されると起動します。**
```bash
ros2 launch erasers_kachaka_manipulation manipulation_launch.py
```

# セットアップ
　erasers_kachaka_manipulation を使用するには
[**erasers_manipulation をセットアップする方法**](/doc/manipulation.md)
を参照してください。

# Build
```
colcon build --symlink-install --packages-select erasers_kachaka_manipulation
```
or
```
colcon build --symlink-install --packages-up-to erasers_kachaka_manipulation
```
