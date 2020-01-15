## セットアップ方法
### 必要なもの
- Eigen

``` bash
sudo apt install sudo apt install libeigen3-dev 
```

- yaml-cpp

``` bash
sudo apt install libyaml-cpp-dev
```

- OpenCV 3.4.9

`cmake-gui ..`を実行すると設定Windowが開く。
1. `Configureボタン`をおす。
2. `extra`で検索。
3. 出てきたパスを入れれそうなところに`opencv_contrib/modules`までのフルパスを入力。
4. もう一回、`Configureボタン`を押す。
5. 最後に`vtk`で検索して、`DIR_PATH`がNOT_FOUNDになってなければ、OK
6. `Generateボタン`を押してWindowを閉じる。

``` bash
# 依存関係
sudo apt install libvtk6-dev
sudo apt install cmake-qt-gui
# ソースコードをダウンロードする
wget https://github.com/opencv/opencv/archive/3.4.9.zip -O opencv3.4.9.zip
wget https://github.com/opencv/opencv_contrib/archive/3.4.9.tar.gz -O opencv_contrib3.4.9.zip

unzip opencv3.4.9.zip
tar zxvf opencv_contrib3.4.9.zip

cd opencv3.4.9
mkdir build
cd build
cmake-gui ..
make -j13
sudo make install
```

