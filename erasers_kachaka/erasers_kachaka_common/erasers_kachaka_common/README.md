# erasers_kachaka_common
　erasers_kachaka の各種機能を実装、各種 API を実装するパッケージです。このパッケージから起動するノードは以下の通りです。

 - **kachaka_speak_subscriber**<br>
   トピック `.../kachaka_speak` 経由でカチャカから発話させるノードです。詳しくは
   [トピックからカチャカを発話させる方法](/docs/howtospeak.md#topic)
   を参照してください。
 - **emergency_manager**<br>
   緊急停止を管理するノードです。詳しくは
   [サービスから緊急停止させる方法](/docs/howtoemc.md)
 - **lidar_observer**<br>
   カチャカは通常一定時間車輪制御が行われないと節電のため Lidar センサーなど各種センサーが停止します。これを回避するためのノードです。

　このパッケージから利用できる API は以下の通りです。

- **`erasers_kachaka_common.erasers_kachaka_common.tts.TTS`**<br>
  カチャカから発話させる API です。詳しくは
  [common API からカチャカを発話させる方法](/docs/howtospeak.md#api)
  を参照してください。

# Build
```
colcon build --symlink-install --packages-select erasers_kachaka_common
```
or
```
colcon build --symlink-install --packages-up-to erasers_kachaka_common
```
