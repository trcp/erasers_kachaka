# erasers_kachaka_common
　erasers_kachaka に拡張機能と **API** を提供するパッケージです。

## common API
　ここでは、当パッケージの API の使い方を解説します。この API を使うことで効率的に erasers_kachaka を制御、開発することができます。
 
### `tts.TTS`
　このモジュールはカチャカからの発話（Text-To-Speech）を実装します。以下は当モジュールのリファレンスです。
```python
TTS.say(text="引数テキストに発話させたい文字列を代入してください。", wait=True)
```
- **`text`**<br>
        デフォルト値 : $`\text{引数テキストに発話させたい文字列を代入してください。 }`$<br>
        **Type**= $`\text{String}`$<br>
        発話させたいテキストを代入してください。代入されたテキストを kachaka から発話します。数字のみ読み上げさせたい時も代入する型を String 型に変換してください。

- **`wait`**<br>
        デフォルト値 : $`\text{True }`$<br>
        **Type**= $`\text{Bool}`$<br>
        引数`text`に代入したテキストの発話が完了するまで待機するかどうかを設定する引数です。デフォルトの場合は発話が完了するまで待機し、プロセスは一時停止します。この引数に`False`を代入すると発話が完了するまで待機はせず、そのままプロセスを継続します。

---

以下のサンプルコードは `TTS` モジュールのいくつかの使用例を示します。
```python
#!/usr/bin/env/python3
import rclpy

# import this module
from erasers_kachaka_common.tts import TTS

# initialize rclpy
rclpy.init()

# initialize TTS
tts = TTS()

# Basic TTS usage
tts.say("Hello world!")
print("said Hello world!")

# Not wait tts comlete
tts.say("Hello world!", False)
print("saying Hello world!")
```
`TTS`モジュールを使用するには ROS2 Python API `rclpy`を事前に初期化する必要があります。必ず事前に`rclpy`をインポートし、`rclpy.init()`を行ってください。
```python
import rclpy
...
rclpy.init()
```
`TTS`の`say`メソッドを使用して TTS を実装します。任意の変数を使い`TTS`クラスをインスタンス化してください。ここでは変数`tts`をインスタンスとして使用します。
```python
tts = TTS()
```
`say`メソッドに発話させたいテキストを代入します。以下のコードが最も基本的な`TTS｀モジュールの使用方法です。代入されtテキストの発話が完了するまでプロセスは待機します。
```python
tts.say("Hello world!")
```
第2引数`wait`に`False`を代入することで プロセスを待機せずに発話します。
```python
tts.say("Hello world!", False)
```
