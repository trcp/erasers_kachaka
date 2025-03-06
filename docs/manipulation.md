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

## 起動

1. **電源を確認する**<br>
   erasers_kachaka シェルフ底部にある Omnicharge バッテリーの AC ボタンが光っていることを確認してください。光っていない場合はバッテリーの AC ボタンを押してください。
2. **緊急停止ボタンを確認する**<br>
   シェルフ上部、アーム側にある緊急停止ボタンを引いて、緊急停止を解除してください。
3. **起動に最適な姿勢にする**<br>
   アームを以下の画像のような姿勢にしてください。アームの手先がシェルフ手前にぶら下がるような姿勢です。<br>
   <img src="/imgs/manipulation_pose.png" width=50% />

   **以下のような姿勢にはしないでください！アームが正常に起動しません。**<br>
   <img src="/imgs/manipulation_ng_pose0.png" width=30% /><img src="/imgs/manipulation_ng_pose1.png" width=30% /><img src="/imgs/manipulation_ng_pose2.png" width=30% />
   
5. **マニピュレーターを起動する**<br>
   すでに erasers_kachaka_bringup で erasers_kachaka が起動しているなら、以下のコマンドを実行してマニピュレーターを起動します。
   ```bash
   ros2 launch erasers_kachaka_manipulation manipulation_launch.py
   ```
   1回のコマンドで erasers_kachaka、マニピュレーターを同時に起動したい場合は以下のコマンドを実行してください。
   ```bash
   ros2 launch erasers_kachaka_bringup bringup.launch.py shelf_type:=2
   ```
   起動すると、マニピュレーターは自動できに **ホームポジション** に移動します。もしアームが自動的にホームポジションに移動しない場合、なんらかの問題が発生して正常にマニピュレーターが起動していないことを示しています。

## 操作方法
　マニピュレーターを起動すると、専用の Rviz インターフェースが起動します。

## サービスを使用する

## API を使用する
