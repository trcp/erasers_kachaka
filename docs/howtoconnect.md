# カチャカと接続する方法
<a id='ethernet'></a>
## 有線接続する方法
　erasers_kachaka は **GL.iNet ルーター** を使用してカチャカと有線接続を行っています。
<p align="center">
<img width=25% src='https://m.media-amazon.com/images/I/41IiLQJghoL.jpg'/><br>
Gl.Net ルーター
</p>

カチャカは現在以下の表に基づいて IP アドレスが振られています。カチャカの番号はカチャカの LED リング内に記述されています。

<a id='table'></a>
|||
|:---:|:---|
|kachaka 1|192.168.8.11|
|kachaka 2|192.168.8.12|
|kachaka Pro|192.168.8.13|

---
0. **Gl.Net とバッテリーを接続する**<br>
  　Omnicharge バッテリーを用意し、USB Type-C 経由で GL.Net ルーターを接続します。
  
  <p align="center">
  <img width=25% src='https://jp.omnicharge.co/cdn/shop/files/omnicharge-omni-20_be168e67-5aea-4ca2-81a1-9c37e7cba3d0.jpg?v=1723394929' />
  <br>
  Omnicharge バッテリー
  </p>
  
  　Omnicharge バッテリーの電源を入れると、ルーターの電源も入ります。ルーターのランプが光ったら事を確認してください。

2. **コンピューターと Gi.iNet ルーターを Ethernet 経由で接続する**<br>
  　ルーター背面に空いている Lan ポートに Ethernet ケーブルを差し込み、コンピューターとルーターを接続します。コンピューターのデスクトップ画面右上に Ethernet が接続されていることを示すマークまたは項目があれば OK です。

3. **環境変数 `KACHAKA_IP` に IP アドレスを与える**<br>
   例えば、カチャカの IP アドレスが `192.168.8.11` の場合、以下のコマンドを実行して環境変数 `KACHAKA_IP` に IP アドレスを登録します。
   ```bash
   export KACHAKA_IP=192.168.8.11
   ```
   または、`~/.bashrc` に上記コマンドを直接書き込んでください。~/.bashrc を編集する際は、以下のコマンドを実行します。
   ```bash
   gedit ~/.bashrc
   ```
   ~/.bashrc 一番下の `export KACHAKA_IP=...` に接続されているカチャカの IP アドレスを入力してください。現在接続されているカチャカの IP アドレスはこのドキュメントの上にある
   [テーブル](#table)
   を参照してください。<br>
   　編集がおわったら gedit を終了して以下のコマンドを実行し変更を反映します。
   ```bash
   source ~/.bashrc
   ```
5. **接続確認**<br>
   以下のコマンドを実行してカチャカと通信できるかどうか確認してください。
    ```bash
    ping $KACHAKA_IP
    ```

---

　これで
[erasers_kachaka を起動する](/erasers_kachaka/erasers_kachaka_bringup/README.md)
作業を行えば、カチャカを起動できます。

<a id='wireless'></a>
## 無線接続する方法

1. **カチャカを Wi-Fi に接続する**<br>
  カチャカの LED リングライトがたまに紫色の点滅する場合、カチャカが Wi-Fi に接続されていないことを示しています。カチャカを Wi-Fi に接続する方法は
  [Wi-Fi再設定](https://kachaka.zendesk.com/hc/ja/articles/7032634306575-Wi-Fi%E5%86%8D%E8%A8%AD%E5%AE%9A)
  を参照してください。

> [!NOTE]
> コンピューターとカチャカが同じ Wi-Fi に接続されていることを確認してください。

2. **カチャカの IP アドレスを取得する**<br>
   [アプリ上で確認する](https://kachaka.zendesk.com/hc/ja/articles/7873356524559-%E3%82%AB%E3%83%81%E3%83%A3%E3%82%AB%E3%81%AEIP%E3%82%A2%E3%83%89%E3%83%AC%E3%82%B9%E3%81%AE%E7%A2%BA%E8%AA%8D%E6%96%B9%E6%B3%95)
   または、カチャカに向かって、「ねぇカチャカ。IP アドレスを教えて」と話して、カチャカの IP アドレスを取得してください。

3. **環境変数 `KACHAKA_IP` に IP アドレスを与える**<br>
   例えば、カチャカの IP アドレスが `192.168.8.11` の場合、以下のコマンドを実行して環境変数 `KACHAKA_IP` に IP アドレスを登録します。
   ```bash
   export KACHAKA_IP=192.168.8.11
   ```
   または、`~/.bashrc` に上記コマンドを直接書き込んでください。

 4. **接続確認**<br>
   以下のコマンドを実行してカチャカと通信できるかどうか確認してください。
    ```bash
    ping $KACHAKA_IP
    ```

---

　これで
[erasers_kachaka を起動する](/erasers_kachaka/erasers_kachaka_bringup/README.md)
作業を行えば、カチャカを起動できます。
