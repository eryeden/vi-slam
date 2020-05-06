# OpenGVメモ

## 基本用語
- Bearing vector

カメラ座標系での単位ベクトルで３次元ポイント方向を指す。
よって２自由度。カメラ座標系での定義が多い。
単位ベクトルなので位置を表すわけではなさそう。

単位球面にLandmark位置を投影した形になるので、VINS-Monoと同じ扱いをしている。
カメラモデルとの関係では、画像自体が平面に補正できなくても、特徴点位置のみが補正して、
単位球面上に観測Feature point位置を投影できればOK。

- Landmark

３次元の点を示す。
この用語だと、世界座標系（world reference frame）での３次元点を示すことが多い。

- Camera

OpenGVではキャリブレーションされたカメラしか登場しないので、
Landmarkの観測結果は、Bearing vectorとカメラ座標系の定義だけから表現することができる。

なので、Bearing vectorのところでも書いたけど、全ての観測結果は、カメラ座標系の
原点を中心とした単位球面上に存在することになる。

- Viewpoint

複数カメラを扱えるようにする概念。
VPを利用すると、VP座標系とVPに含まれるカメラ座標系、それぞれのカメラ座標系における
観測を表現するBearing vectorからOpenGVが構成されることになる。
シングルカメラも厳密には、VP座標系とカメラ座標系が一致している状態として表現される。
マルチカメラリグなど表現できるのでいつか使ってみたい。

- Pose

ViewpointのPositionとOrientationを、固定した参照座標系上で定義したもの。
参照座標系は、世界座標系か、ほかのViewpointを使って相対的なPoseを表現してもよし。

- Absolute Pose

世界座標系で定義されるPoseを示す。

- Relative Pose

ほかViewpointからの相対的Poseを示す。

- Correspondence

同じLandmarkを示す対応のこと。以下の対応がある。
複数Viewpoint間のBearing vectorの対応（2D-2D）、
Bearing vectorと３次元点の対応（2D-3D）、
異なる座標系間の３次元点の対応（3D-3D）

## ライブラリの使い方

Keywordとして、"Adapter"と"Visitor"があるらしい。

- 

