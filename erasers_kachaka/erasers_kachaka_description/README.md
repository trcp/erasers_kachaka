# erasers_kachaka_description
　erasers_kachaka のロボットモデルをスポーンさせるパッケージです．以下の３つのオプションモデルを提供します．

|Kachaka 単体|Kachaka + ２段シェルフ|Kachaka + ３段シェルフ|
|:---:|:---:|:---:|
|<img src="https://i.imgur.com/GqBfB6Q.png"/>|<img src="https://i.imgur.com/9MUNfMY.png"/>|<img src="https://i.imgur.com/7ds35UQ.png"/>|

## 使用方法
`debug` 引数を有効にすることでロボットモデルを確認することができます．

- kachaka 単体
    ```bash
    ros2 launch erasers_kachaka_description description.launch.py debug:=true use_shelf:=false
    ```
- kachaka + ２段シェルフ
    ```bash
    ros2 launch erasers_kachaka_description description.launch.py debug:=true use_shelf:=true
    ```
- kachaka + ３段シェルフ
    ```bash
    ros2 launch erasers_kachaka_description description.launch.py debug:=true use_shelf:=true shelf_type:=3
    ```


# Build
```
colcon build --symlink-install --packages-select erasers_kachaka_description
```
or
```
colcon build --symlink-install --packages-up-to erasers_kachaka_description
```
