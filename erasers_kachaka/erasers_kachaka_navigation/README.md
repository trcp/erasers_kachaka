# erasers_kachaka_navigation
　ROS2 Navigation Stack でナビゲーションと SLAM を実装するパッケージ。
---
　kachaka は標準でロボット本体のナビゲーションシステムによりマッピング及びナビゲーションを行います。このパッケージはホストコンピューター上でこれらのタスクを実装し、拡張性を向上させます。<br>
　このパッケージでは cartographer と Navigation2 を使用します。

## SLAM を実行する
　SLAM を実行するには、`cartographer.launch.py` をつかいます。
