# erasers_kachaka_bringup

　erasers_kachaka の起動を司るパッケージです．

## ビルドする

> [!NOTE]
　以下の作業はビルドされていない場合，または２次開発後のみ実行します．

　手動ビルドには以下のコマンドを **ワークスペースディレクトリ内で** 実行してください．
```bash
colcon build --symlink-install --packages-up-to erasers_kachaka_bringup
```

## erasers_kachaka 起動方法
　Docker を使用している場合とローカル環境で ROS2 から直接起動させている場合で若干操作方法が異なります．

<details>
<summary>
Docker を使用している場合
</summary>

1. **erasers_kachaka 内にある [kachaka_env](/kachaka_env) を編集します．**<br>
    このファイルの `KACHAKA_IP` に記述されている IP アドレスを接続したい kachaka の IP アドレスに変更してください．
    ```
    KACHAKA_IP=<接続先の IP アドレス>
    ```

1. **erasers_kachaka コンテナと Bridge コンテナを起動する**<br>
    以下のコマンドを実行して erasers_kachaka と Bridge コンテナを起動します．
    ```
    # 推奨
    # Kachaka 内蔵マップなしで起動したい場合
    docker compose --env-file kachaka_env up erasers_kachaka nomap_bridge
    ```
    ```
    # Kachaka 内蔵マップ有りで起動したい場合
    docker compose --env-file kachaka_env up erasers_kachaka official_bridge
    ```
</details>

<details>
<summary>
ローカル環境の場合
</summary>
</details>
