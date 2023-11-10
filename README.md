# aquestalkpi_ros

## AquesTalk Pi 本体の導入

[AquesTalk Pi 公式サイト](https://www.a-quest.com/products/aquestalkpi.html)から AquesTalk Pi 本体をダウンロードし、任意のパスに展開します。

64 ビット 環境の場合は、`aquestalkpi/bin64`ディレクトリ内にある実行ファイル`AquesTalkPi`を、`aquestalkpi`ディレクトリ内の同名ファイルに上書きしてください(32 ビット環境の場合、この操作は不要です)。

※AquesTalk Pi の使用には、個人の非営利目的を除いて「使用ライセンス」が必要です。詳しくは、[AquesTalk Pi 公式サイト](https://www.a-quest.com/products/aquestalkpi.html)をご覧ください。

## 依存関係の解決

```bash
git clone https://github.com/eyr1n/aquestalkpi_ros.git
rosdep install -yi --from-paths aquestalkpi_ros
```

## ビルド・実行

パラメータ`aquestalkpi_path`に AquesTalk Pi へのパスを渡します(フルパスがおすすめ)。

```bash
colcon build
source install/setup.bash
ros2 run aquestalkpi_ros aquestalkpi_ros --ros-args -p aquestalkpi_path:=[AquesTalk Piへのパス]
```

以下のようなメッセージを publish するとしゃべります。

```bash
ros2 topic pub /aquestalkpi_ros/talk aquestalkpi_ros_msgs/msg/Talk '{text: こんにちは, voice: f1, speed: 100}'
```
