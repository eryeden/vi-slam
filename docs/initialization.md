## Visual-SLAM 初期化関係の実装

### VITAMIN-Eでは？
Essential行列の分解で２フレーム間のカメラ移動と回転を求めている。
とりあえず、OpenCVのFindEssentialMatを使ってみて初期化を試してみる。
同一平面上に特徴点が存在する場合など基礎行列が求まらない場合などあるらしい？（F matrixの歌によると）のでそういった場合は別途体操が必要かもしれない。Homographyを利用する必要があるかもしれない。

### OrbSLAMでは？
Homography行列と基礎行列を両方求めることで初期化する。


### Essential行列の推定とその分解について
推定はOpenCVを使ってみる。分解はどうする？

#### 分解について
``` c++
decomposeEssentialMat(InputArray E, OutputArray R1, OutputArray R2, OutputArray t)
```
これで、Eを分解すると、R1、R2、tがでてきて、４パターン生まれる。
[R1,t], [R1,−t], [R2,t], [R2,−t]になるらしい。これから適切なやつを選ぶ必要がある。
どっちに進んでいるかわかればTの選定ができる？

どうやら、Eと特徴点の対応状況を利用することで二枚の画像間の相対的な位置関係を選ぶものがあるらしい。
``` c++
int cv::recoverPose	(	InputArray 	E,
                        InputArray 	points1,
                        InputArray 	points2,
                        InputArray 	cameraMatrix,
                        OutputArray 	R,
                        OutputArray 	t,
                        InputOutputArray 	mask = noArray() 
)		
```
参考：https://docs.opencv.org/master/d9/d0c/group__calib3d.html#ga7f60bdff78833d1e3fd6d9d0fd538d92

OpenCVでも実装できそうだが、OpenGVにもいろいろ関数が実装されているらしい。
ちまたのV-SLAMもOpenGVを使っていることあり。Kimera-VIOなど。
いろいろライブラリはあるので、ひとまずOpenCVでいろいろやってみて様子見する。
このあとのBA部分がとても重いと思われるので。

### V-SLAM初期化関係
- 対応した特徴量からのホモグラフィー行列の算出
- ホモグラフィー行列（H）の分解（カメラの併進、回転行列を計算する）
- ホモグラフィーと基礎行列（F）の違いはなんなのか？
- ホモグラフィーでの初期化、基礎行列での初期化、Eでの初期化？などいろいろあるがどれでやればいいの？
- この話でエピポーラの話はどこに登場するの？
- そもそも初期化はなんの変数を初期化するのか？（カメラのパラメータだけ？）
- 二枚の画像間のカメラPoseをHやFやEから求めて、その求めた移動量から特徴点の３D位置を三角測量するという認識で正しいか？
- OrbSLAMなど初期化方法を何種類かトライしている。なにを利用しているのか？

### 描画について
- Pangolin : https://github.com/stevenlovegrove/Pangolin
- OpenCV::viz : https://docs.opencv.org/master/d7/df9/tutorial_table_of_content_viz.html
- OpenGV? : https://github.com/laurentkneip/opengv

### リンク集
- [３次元復元関係の用語についていい感じのまとめ](https://medium.com/@NegativeMind/2d-3d%E5%BE%A9%E5%85%83%E6%8A%80%E8%A1%93%E3%81%A7%E4%BD%BF%E3%82%8F%E3%82%8C%E3%82%8B%E7%94%A8%E8%AA%9E%E3%81%BE%E3%81%A8%E3%82%81-27403689da1b)
- [エピポーラについて](https://qiita.com/ykoga/items/14300e8cdf5aa7bd8d31)
- [基礎行列の歌](https://www.youtube.com/watch?v=EQi_B3VFGHY)

