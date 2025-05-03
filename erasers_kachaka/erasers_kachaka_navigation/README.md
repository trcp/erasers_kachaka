# erasers_kachaka_navigation
　ナビゲーション用パッケージです。以下のコマンドを実行してナビゲーションを実行します。
```bash
ros2 launch erasers_kachaka_navigation navigation_launch.py
```

## pot_fields
　ポテンシャル・フィールド法を用いた障害物に対する**反発力**をもとめる。

## leg_finder
　人の足を検出するノード。

# Build
```bash
rosdep install -i -y --from-path src/erasers_kachaka/erasers_kachaka/erasers_kachaka_navigation
```
```bash
colcon build --symlink-install --packages-up-to erasers_kachaka_navigation
```
or
```bash
colcon build --symlink-install --packages-select erasers_kachaka_navigation
```
