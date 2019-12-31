## 特徴点抽出について
開発方針としては、はじめに思いついたとおりに実装してみる。次に、抽象化したクラスに落とし込むようにする。

要パフォーマンスチェック。

### プレ実装
#### 曲率画像の生成
OpenCVの画像演算系を使って実装した。パフォーマンスの計測は指定いないが、結構重そうな感じがする。
- 参考：http://opencv.jp/opencv-2svn/cpp/operations_on_arrays.html#cv-reduce

#### 曲率ピークピック
画素中心からの８近傍探索を繰り返すことでピークピックを行う。
時間がかかるかを思っていたが、意外と処理できている。OpenMPかなにかで並列化できればより高速に処理できると思われる。
- 参考：https://stackoverflow.com/questions/23274906/opencv-fast-mat-element-and-neighbour-access


#### Dominant flow estimation
ここではopencv_contribに入っているFeature descripterを利用している。
なのでOpenCV contribも入れておく必要あり。以下メモ。

1. ダウンロード：今はバージョン、3.4.9を利用するのでこのソースコードを落とす。
``` bash
wget https://github.com/opencv/opencv/archive/3.4.9.tar.gz
wget https://github.com/opencv/opencv_contrib/archive/3.4.9.tar.gz
```

2. もろもろ設定してコンパイル
``` bash
tar zxvf opencv-3.4.9.tar.gz
tar zxvf opencv_contrib-3.4.9.tar.gz

cd opencv-3.4.9
mkdir build; cd build
cmake-gui .. # EXTRAで検索して/path/to/opencv_contrib/modulesを設定、もう一回configureする。
make -j12
sudo make install
```
