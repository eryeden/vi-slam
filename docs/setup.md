## セットアップ方法
うまく行かなかったら[Issue投稿](https://github.com/eryeden/vi-slam/issues/new)頼みます。😊

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

- [spdlog](https://github.com/gabime/spdlog)

```bash
# ソースコードをダウンロード
cd somewhere/convenient/directory/
wget https://github.com/gabime/spdlog/archive/v1.5.0.tar.gz
tar zxvf v1.5.0.tar.gz
cd spdlog-1.5.0
mkdir build
cd build
cmake .. -DCMAKE_CXX_FLAGS="-fpic" #このオプションがないと下記エラーがでるので注意。
make -j; sudo make install
```
Error message:
```
/usr/bin/ld: /usr/local/lib/libspdlog.a(spdlog.cpp.o): relocation R_X86_64_TPOFF32 against `_ZGVZN6spdlog7details2os9thread_idEvE3tid' can not be used when making a shared object。 -fPIC を付けて再コンパイルしてください。
```

- [fmt](https://github.com/fmtlib/fmt)

ref: https://github.com/fmtlib/fmt/blob/master/doc/usage.rst
```bash
# ソースコードをダウンロード
cd somewhere/convenient/directory/
wget https://github.com/fmtlib/fmt/archive/6.2.0.tar.gz
tar zxvf 6.2.0.tar.gz
cd fmt-6.2.0
mkdir build
cd build
cmake ..
make -j; sudo make install
```

- [opengv](https://github.com/laurentkneip/opengv)

ref: https://laurentkneip.github.io/opengv/page_installation.html
```bash
sudo apt-get install build-essential
sudo apt-get install cmake
sudo apt-get install cmake libeigen3-dev

# Downloading the source code
git clone https://github.com/laurentkneip/opengv
# Go to the top-level directory of OpenGV. Type:
mkdir build && cd build && cmake .. && make
sudo make install
```
