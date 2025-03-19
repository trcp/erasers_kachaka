# カチャカから発話させる方法
　erasers_kachaka から発話させる方法を解説します。

<a id=api></a>
## common API からカチャカを発話させる方法
　erasers_kachaka_common の **Python API** を使いカチャカを発話させる方法を解説します。

1. **Python スクリプトファイルの作成**
2. **必要なモジュールのインポート**<br>
  以下のモジュールをインポートします。
    ```python
    # ROS modules
    from rclpy.node import Node
    import rclpy
    
    # Speaker API
    from erasers_kachaka_common.tts import TTS
    ```
4. **`rclpy` の初期化とノード宣言**<br>
　発話クラス TTS は ROS2 インターフェースを必要とします。以下のコードを参考にし、rclpy を初期化し、ノード宣言を行なってください。
    ```python
    # initialize rclpy
    rclpy.init()
    
    # create ROS2 node
    node = Node("sample_kachaka_speak")
    ```
6. **TTS クラスの初期化**<br>
  TTS クラスを初期化する際に TTS クラスの引数に先ほど初期化したノードインスタンスを代入してください。これを行わないとエラーになります。
    ```python
    # create ROS2 node
    node = Node("sample_kachaka_speak")
    
    # initialize TTS
    tts = TTS(node)
    ```
8. **発話する**<br>
  TTS のメソッド `say` の第1引数に発話したいテキストを文字列で代入してください。このプログラムを実行すると発話が行われます。
    ```python
    # speak!
    tts.say("Hello kachaka!")
    ```

---

　サンプルコードの実行と確認は
[`erasers_kachaka/erasers_kachaka/erasers_kachaka_common/samples/kachaka_speak.py`](/erasers_kachaka/erasers_kachaka_common/samples/kachaka_speak.py)
を参照してください。

<a id=topic></a>
## トピックからカチャカを発話させる方法
　ROS2 トピックからカチャカを発話させるメッセージを送ることでも発話させることができます。ターミナルで以下のコマンドを実行してみましょう。するとカチャカから "Hello Kachaka" と発話します。
```bash
ros2 topic pub --once /$KACHAKA_NAME/kachaka_speak std_msgs/msg/String "{data: Hello Kachaka!}"
```

- [トップに戻る](/README.md)
