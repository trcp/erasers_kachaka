# tts
　カチャカから言葉を発話させるモジュールです。コードに以下のようにモジュールをインポートします。
```python
from erasers_kachaka_common.tts import TTS
```
　このモジュールは rclpy に依存しているため、rclpy もインポートします。
```python
import rclpy
```
　TTS モジュールをインスタンス化する前に rclpy を初期化してください。
```python
rclpy.init()
```
　インポートしたモジュール **`TTS`** を以下のように任意の変数に格納してインスタンス化します。
```python
tts = TTS()
```
　以下のように `say` メソッドを利用できるようになります。say メソッド内に発話させたいテキストを代入してください。
```python
tts.say("こんにちは。erasers kachaka !")
```
　以下がサンプルコードです。参考にしてみてください。
```python
from erasers_kachaka_common.tts import TTS
import rclpy

rclpy.init()

tts = TTS()
tts.say("こんにちは。erasers kachaka !")
```

> このサンプルコードを実行するとカチャカから *「こんにちは。erasers kachaka !」* と発話します。

# リファレンス
```python
erasers_kachaka.common.TTS(timeout: float=10.0) -> None
```
|Argument|type|Default Value|Description|
|:---:|:---:|:---:|:---|
|**timeout**|$`\text{float}`$|$10.0$|TTS サービスの待機時間を指定します。指定された時間以降 TTS サービスが起動しない場合エラーログを出力します。|

---

```python
erasers_kachaka.common.TTS.say(text: str="引数テキストに発話させたい文字列を代入してください。",
                               wait: bool=True
                              ) -> bool
```
|Argument|type|Default Value|Description|
|:---:|:---:|:---:|:---|
|**text**|$`\text{str}`$|`"引数テキストに発話させたい文字列を代入してください。"`|発話させたいテキストを入力します。|
|**wait**|$`\text{bool}`$|$`\text{True}`$|発話が完了するまで待機します。False にすると発話の完了を待機せず次のプロセスは移行します。|

|Return Type|Description|
|:---:|:---|
|$`\text{bool}`$|発話が完了すると True を返します。失敗すると False を返します。`wait` が False の場合、例外なく True を返します。|

# エラー
## [ERROR] Service service_tts is not working !
　このエラーはサービス service_tts が起動していない場合に発生します。以下のコマンドを実行して直接 service_tts を起動させるか、brinugup を起動してください。
```bash
ros2 run erasers_kachaka_common service_tts
```
