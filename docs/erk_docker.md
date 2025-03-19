# ros2_bridge kachaka Docker コンテナの起動チェック
　ここでは **erasers_kachaka の主要機能を使わずに kachaka との通信のみ行いたい場合** や、**トラブルシューティングにおける対処法** を解説します。

## 現在のコンテナの状態を確認する方法
　[erasers_kachaka を起動](/docs/howtobringup.md)
すると、kachaka との通信用 Docker コンテナが起動します。起動中のコンテナの状態を確認したい場合、`erasers_kachaka/docker` ディレクトリ上で以下のコマンドを実行してください。
```bash
docker compose logs kachaka
```
　上記コマンドを実行すると kachaka と ROS2 間の通信に関するコンポーネントのステータスを確認できます。**`[ERROR]`** と表示されている場合、なんらかのエラーが発生しているので
 [**コンテナを直接再起動する**](#reboot)
 か、その他対策を行なってください。

 ## コンテナのみ起動したい場合
 　erasers_kachaka の各種機能を使用せず、kachaka との通信のみ行いたい場合、`erasers_kachaka/docker` ディレクトリ上で以下のコマンドを実行してください。
```bash
docker compose up kachaka
```

> [!TIP]
> コンテナを起動したターミナル上で作業したい場合、以下のコマンドのように オプション `-d` をつけて実行するとログが表示されません。
> ```bash
> docker compose up -d kachaka
> ```

<a id="reboot"></a>
## コンテナを再起動したい場合
　なんらかのトラブルや状況で **コンテナのみ再起動したい** 場合、`erasers_kachaka/docker` ディレクトリ上で以下のコマンドを実行してください。
```bash
docker compose restart kachaka
```
