# CRANE_X7 セットアップ方法

## ダウンロード

1. **repos の追加**<br>
   　以下のコマンドを実行して `colcon_ws` に移動してください。
   ```bash
   cd ~/colcon_ws
   ```
   以下のコマンドを実行して CX7 用のリポジトリをワークスペースに追加します。
   ```bash
   vcs import src < src/erasers_kachaka/manipulation.repos
   ```
3. **依存関係の解決**<br>
    　以下のコマンドを実行して依存関係の解決を行います。
   ```bash
   rosdep install -y -i --from-path src/er_crane_x7
   ```

## ビルド
  以下のコマンドを実行して CX7 関連のパッケージのみビルドします。
 ```bash
 colcon build --symlink-install --packages-up-to erasers_kachaka_manipulation
 ```
