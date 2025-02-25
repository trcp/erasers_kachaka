# erasers_kachaka_cartographer
　マップを作成するパッケージです。以下のコマンドを実行してマップを作成できます。
```bash
ros2 launch erasers_kachaka_cartographer cartographer_launch.py
```

# Build
```bash
rosdep install -i -y --from-path src/erasers_kachaka/erasers_kachaka/erasers_kachaka_cartographer
```
```bash
colcon build --symlink-install --packages-up-to erasers_kachaka_cartographer
```
or
```bash
colcon build --symlink-install --packages-select erasers_kachaka_cartographer
```