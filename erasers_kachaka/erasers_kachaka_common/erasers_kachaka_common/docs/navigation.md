# navigation
　カチャカを指定された座標へ移動させるモジュールです。コードに以下のように SimpleNavigator モジュールをインポートします。
```python
from erasers_kachaka_common.navigation import SimpleNavigator
```
 SimpleNavigator モジュールはカチャカをナビゲーションで制御するための簡単な制御メソッドが用意されています。ここではこのモジュールを使用してカチャカをナビゲートしてみます。
このモジュールは rclpy に依存しているため、rclpy もインポートします。
```python
import rclpy
```
　SimpleNavigator モジュールをインスタンス化する前に rclpy を初期化してください。
```python
rclpy.init()
```
　インポートしたモジュール **`SimpleNavigator`** を以下のように任意の変数に格納してインスタンス化します。
```python
navigation = SimpleNavigator()
```

> [!CAUTION]
> SimpleNavigator を使用する前にカチャカを揺さぶるか速度司令を与えるなどしてカチャカの Lidar センサーをアクティブにする必要があります。この問題はいづれ自動化します。

## 現在位置を取得する
　カチャカの現在位置を取得するには、get_current_pose を使用します。以下の例では取得したカチャカの現在位置を [PoseStamped](https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html) 形式で取得します。
 ```python
pose = navigation.get_current_pose()
# PoseStamped.pose.x = XXXX
# PoseStamped.pose.x = YYYY
# ...
```
PoseStamped 形式ではなく、[x座標、y座標、Yaw軸] で構成された配列データで取得したい場合は、以下のように `xyy` 引数に True を代入します。
```python
pose = navigation.get_current_pose(xyy=True)
# [x, y, yaw]
```
> [!TIP]
> `xyy=True` 時に取得される配列インデックス2（yaw）は弧度法で示されます。

## マップ基準でナビゲーションする
　map フレーム基準でカチャカを目標地点までナビゲーションするには、go_abs を使用します。以下の例ではカチャカを map フレーム基準で x 座標に 1.0m 移動させます。
```python
navigation.go_abs(x=1.0)
```
y 座標にも移動させたい場合はこのように引数 y に値を入れましょう。
```python
navigation.go_abs(x=1.0, y=1.0)
```
目標座標の角度を設定したい場合はこのように引数 yaw に弧度法で入力しましょう。
```python
navigation.go_abs(x=1.0, y=1.0, yaw=1.57) # 90 度
```
目標座標の角度を度数法で設定したい場合は引数 degrees を True にします。
```python
navigation.go_abs(x=1.0, y=1.0, yaw=90.0, degrees=True)
```
> [!CAUTION]
> degrees 以外の引数は浮動小数点型で入力してください。

引数に値を何も入れない状態で実行すると、カチャカは map の中央へ移動します。
```python
navigation.go_abs()
```

## ロボット基準でナビゲーションする
　ロボット基準でカチャカを目標地点までナビゲーションするには、go_rlt を使用します。以下の例ではカチャカを ロボット基準で x 座標に 1.0m 移動させます。
```python
navigation.go_rlt(x=1.0)
```
y 座標にも移動させたい場合はこのように引数 y に値を入れましょう。
```python
navigation.go_rlt(x=1.0, y=1.0)
```
目標座標の角度を設定したい場合はこのように引数 yaw に弧度法で入力しましょう。
```python
navigation.go_rlt(x=1.0, y=1.0, yaw=1.57) # 90 度
```
目標座標の角度を度数法で設定したい場合は引数 degrees を True にします。
```python
navigation.go_rlt(x=1.0, y=1.0, yaw=90.0, degrees=True)
```
> [!CAUTION]
> degrees 以外の引数は浮動小数点型で入力してください。

## ロボットを旋回させる
　ロボットを旋回させるには rotate を使用します。引数 yaw に旋回させたい角度を入力します。degrees 引数を True にすると度数法で角度を指定することができます。
```python
navigation.rotate(yaw=1.57)
```
