# カチャカを移動させる方法
　このチュートリアルでは **カチャカの移動制御を行う方法** を解説します。erasers_kachaka は主に以下の **3つ** の移動制御方法があります。

- [**`Twist` メッセージを使いカチャカを制御する方法**](#twist)
- [**DefaultNavigation を使ってカチャカを自律走行させる方法**](#default)
- [**Nav2Navigation を使ってカチャカを自律走行させる方法**](#nav2)

---

<a id="twist"></a>
## `Twist` メッセージを使いカチャカを制御する方法
　**`Twist` メッセージ（geometry_msgs/msg/Twist）** を使い、カチャカを **速度司令** により制御することができます。

<a id="default"></a>
## DefaultNavigation を使ってカチャカを自律走行させる方法
　**DefaultNavigation** を使いカチャカを **座標指定** により制御することができます。<br>
　座標指定とは、任意の座標を指定して、そこへロボットを自律走行させることでロボットを制御する方法です。

---

### DefaultNavigation を使用する前に
　DefaultNavgation を使うには、以下の条件が整っている必要があります。

- [x] **カチャカ内部にマップを持っている**
- [x] **erasers_kachaka_bringup で `bringup_type` が $1$ になっている**
- [x] **Rviz2 上にマップが表示されている**
- [x] **カチャカの座標が正しい位置にいる**

### DefaultNavigation のサンプルコードを実行する
　[erasers_kachaka_common の sample ディレクトリ](/erasers_kachaka/erasers_kachaka_common/samples/)
にある [default_navigation.py](/erasers_kachaka/erasers_kachaka_common/samples/default_navigation.py) が DefaultNavigation を使用してカチャカを自律走行させるサンプルコードです。

### DefaultNavigation の使用方法
　DefaultNavigation の使用方法を解説します。まずはじめにパッケージがビルドされているかどうか確認しましょう。以下のコマンドを実行して、`erasers_kachaka_common` パッケージビルドされているかどうか確認しましょう。
```bash
ros2 pkg list | grep erasers_kachaka_common
```

> [!TIP]
> 上記コマンド `ros2 topic list` は、コンピューター上にインストールされている ROS2 パッケージ一覧を表示します。<br>
  そして、`| grep erasers_kachaka_common` コマンドで `ros2 topic list` コマンドから返されるパッケージ一覧に `erasers_kachaka_common` が含まれているかどうか検索します。

> [!CAUTION]
> もし、上記コマンドを実行して赤文字で `erasers_kachaka_common` と表示されない場合、コンピューター上に `erasers_kachaka_common` がインストールされていません。以下のコマンドを実行して `erasers_kachaka_common` をビルドしてください。
> ```bash
> # ~/colcon_ws に移動
> cd ~/colcon_ws
> # erasers_kachaka_common とこのパッケージが依存しているパッケージ全てをビルド
> colcon build --symlink-install --packages-up-to erasers_kachaka_common
> # source を通す
> source ~/colcon_ws/install/setup.bash
> ```

---

　次に Python スクリプトを作成し、DefaultNavigation を使ってみましょう。好きな場所に Python スクリプトを作成してください。そして好きなエディタで作成した Python スクリプトを開いてください。<br>
　それでは以下の手順に従い DefaultNavigation を実装していきましょう。

1. **必要なモジュールのインポート**<br>
    　DefaultNavigation とこれを動かすために必要なモジュールをインポートします。
    ```python
    # DefaultNavigation をインポートする
    from erasers_kachaka_common.navigator import DefaultNavigator
    
    # ROS modules
    from rclpy.node import Node
    import rclpy
    ```
2. **`rclpy`とノードを初期化、宣言する**<br>
    　DefaultNavigator は ROS2 インターフェースを使用するため、`rclpy` を初期化し、ノードを宣言します。
    ```python
    # initialize rclpy
    rclpy.init()
    
    # create ROS2 node
    node = Node("sample_kachaka_default_navigation")
    ```

3. **DefaultNavigation を初期化する**<br>
    　DefaultNavigation を初期化します。インスタンス化させます。初期化時に DefaultNavigation の引数に先ほど宣言した Node オブジェクトを代入する必要があります。
    ```python
    # create ROS2 node
    node = Node("sample_kachaka_default_navigation")
    
    # DefaultNavigation の初期化。node を引数に渡すことを忘れずに。
    navigation = DefaultNavigation(node)
    ```
   これで DefaultNavigation を使う準備が整いました。

---

- **相対座標でロボットを移動させる**<br>
    　相対座標でロボットを移動させるには、DefaultNavigation の以下のメソッドを使用します。
    ```python
    DefaultNavigation.move_rlt(x:float=0.0, y:float=0.0, yaw:float=0.0, wait:bool=True) -> bool
    ```
    　先ほどのプログラムに続けて、以下のように `move_rtl` メソッドを使用します。
    ```python
    # DefaultNavigation の初期化。node を引数に渡すことを忘れずに。
    navigation = DefaultNavigation(node)
    
    # 相対座標でロボットを移動させる
    navigation.move_rlt()
    ```
    　前に $0.5m$ 進むコードを書いてみましょう。相対的に前に $0.5m$ 進むには、このように `move_rlt` メソッドの引数 `x` に $0.5$ を代入します。
    ```python
     # 相対座標でロボットを 0.5m 前に移動させる
    navigation.move_rlt(x=0.5)
    ```
    この状態で作成したプログラムを実行してみましょう。するとロボットが現在座標から x 軸方向へ $0.5m$ 前進します。
  <br><img width=50% src="/imgs/move_rlt.png" /><br>
    $1m$ 前に進みたい場合は引数 `x` に $1.0$ を代入します。この時 **必ず引数に入れる値が foat 型になるように書きましょう。`x=1` と、整数を代入するとエラーになります。**
    ```python
     # 相対座標でロボットを 1.0m 前に移動させる
    navigation.move_rlt(x=1.0)
    ```
    $0.5m$ ロボットを後退させるには負の値を代入します。
    ```python
     # 相対座標でロボットを 0.5m 後方に移動させる
    navigation.move_rlt(x=0.5)
    ```
    　ロボットを左右に移動させたい場合は引数 `y` を使用します。以下のコードを実行するとカチャカは左側に $0.5m$ 移動します。
     ```python
     # 相対座標でロボットを 0.5m 左側に移動させる
    navigation.move_rlt(y=0.5)
    ```
    以下のコードを実行するとカチャカは右側に $0.5m$ 移動します。
     ```python
     # 相対座標でロボットを 0.5m 右側に移動させる
    navigation.move_rlt(y=-0.5)
    ```
    　以下のコードのように、引数 `x` と `y` を組み合わせることでカチャカを任意の座標に移動させることができます。
    ```python
     # 相対座標でロボットを前に 0.5m、左に 0.5m 移動させる
    navigation.move_rlt(x=0.5, y=0.5)
    ```
    　ロボットを旋回させるには引数 `yaw` を使用します。引数 `yaw` には **弧度法** で角度を指定します。以下のコードを実行するとカチャカはその場で $180^{\circ}$ 旋回します。
    ```python
     # 相対座標でロボットを 180° 旋回させる
    navigation.move_rlt(yaw=1.57)
    ```
    　以下のコードのように他の引数と組み合わせることができます。以下のコードを実行すると前に $1.0m$、右に $0.5m$ 移動して $180^{\circ}$ 旋回します。
    ```python
     # 相対座標でロボットを 180° 旋回させる
    navigation.move_rlt(x=1.0, y=-0.5, yaw=1.57)
    ```
  

- **絶対座標でロボットを移動させる**<br>
    　絶対座標でロボットを移動させるには、DefaultNavigation の以下のメソッドを使用します。
    ```python
    DefaultNavigation.move_abs(x:float, y:float, yaw:float, wait:bool=True) -> bool
    ```
    　先ほどのプログラムに続けて、以下のように `move_rtl` メソッドを使用します。
    ```python
    # DefaultNavigation の初期化。node を引数に渡すことを忘れずに。
    navigation = DefaultNavigation(node)
    
    # 絶対座標でロボットを移動させる
    navigation.move_abs()
    ```
    　`move_abs` メソッドは **全ての引数を要求します。** 以下のコードのように全ての引数に $0.0$ を与えるとロボットはマップ座標の原点へ移動します。
    ```python
     # ロボットをマップ原点へ移動させる
    navigation.move_abs(x=0.0, y=0.0, yaw=0.0)
    ```
    以下のように、必要な引数を与えないとエラーになります。
    ```python
     # 引数 yaw を定義し忘れた例
    navigation.move_abs(x=0.0, y=0.0)
    ```
    マップ原点から x 座標に $0.5m$ 前に移動させたい場合、以下のように書いて実行すると、マップ原点から $0.5$ x 軸方向に進んだ位置へロボットが移動します。
    ```python
    navigation.move_abs(x=1.0, y=0.0, yaw=0.0)
    ```
    <br><img width=50% src="/imgs/move_abs.png" /><br>
    マップ原点で回るサンプル
    ```python
    navigation.move_abs(x=0.0, y=0.0, yaw=1.57)
    navigation.move_abs(x=0.0, y=0.0, yaw=1.57)
    ```

<a id="nav2"></a>
## Nav2Navigation を使ってカチャカを自律走行させる方法
