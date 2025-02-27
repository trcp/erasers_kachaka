# カチャカと接続する方法
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

## 有線接続する方法
　erasers_kachaka は **GL.iNet ルーター** を使用してカチャカと有線接続を行っています。現在 **kachaka 1** が IP アドレス `192.168.8.11` で登録されています。今後、以下の表に対応してカチャカの IP アドレスを振ります。

|||
|:---:|:---|
|kachaka 1|192.168.8.11|
|kachaka 2|192.168.8.12|
|kachaka Pro|192.168.8.13|

---
1. **コンピューターと Gi.iNet ルーターを Ethernet 経由で接続する**
2. **環境変数 `KACHAKA_IP` に IP アドレスを与える**<br>
   例えば、カチャカの IP アドレスが `192.168.8.11` の場合、以下のコマンドを実行して環境変数 `KACHAKA_IP` に IP アドレスを登録します。
   ```bash
   export KACHAKA_IP=192.168.8.11
   ```
   または、`~/.bashrc` に上記コマンドを直接書き込んでください。
3. **接続確認**<br>
   以下のコマンドを実行してカチャカと通信できるかどうか確認してください。
    ```bash
    ping $KACHAKA_IP
    ```

---

　これで
[erasers_kachaka を起動する](/erasers_kachaka/erasers_kachaka_bringup/README.md)
作業を行えば、カチャカを起動できます。
