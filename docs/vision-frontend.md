# Vision-frontend

Visual SLAMの処理を分けるとすると、Vision frontendとBcakend。Vision frontendについて調べてみる。
これで、手っ取り早くいまあるSLAMの性能を向上させたい。

## 調査対象
Vision frontendについて調べるが、主に以下の内容について調べる。

- 特徴点検出でのOutlierの出にくい工夫
- 特徴点追跡でのOutlierの出にくい工夫
- Outlierの除去
- Keyframeの選択


## リソース
- [Kiemra-VIO; 2020](https://github.com/MIT-SPARK/Kimera-VIO)
- [Basalt; 2020](https://vision.in.tum.de/research/vslam/basalt)
- [VITAMIN-E; 2019](https://staff.aist.go.jp/shuji.oishi/assets/projects/VITAMIN-E/index.html)
- [xivo; 2019?](https://github.com/ucla-vision/xivo)
- [VINS-Mono; 2017](https://github.com/HKUST-Aerial-Robotics/VINS-Mono)
- [On-Manifold Preintegration for Real-Time Visual-Inertial Odometry; 2016](http://rpg.ifi.uzh.ch/docs/TRO16_forster.pdf)
  - [SVO2.0; 2017: Fast Semi-Direct Visual Odometry](http://rpg.ifi.uzh.ch/svo2.html)
- [DSO; 2016](https://vision.in.tum.de/research/vslam/dso)
- [OrbSLAM; 2015](https://github.com/raulmur/ORB_SLAM2)
- [LSD-SLAM; 2014](https://vision.in.tum.de/research/vslam/lsdslam)


## Outlier除去
世にたくさんあるVisual-SLAMで使っているFrontend手法をまとめてみる。


### Kiemra-VIO; 2020
- Feature detection : Shi-Tomasi corners
- Feature tracking : Lukas-Kanade tracker
- Verification
  - Geometric verification : 5 point RANSAC(Mono), 3 point RANSAC(Stereo)
  - KeyframeのみGeometric verificationを実行

#### 疑問点
- Keyframeの選び方は? Keyframeが適切に選べていればGeometric verificationするときの視差などを考える必要はなくなるはず？
- Monoバージョンが公開されていないが、ここの部分になにか問題はあるのか？
- Keyframeのみで特徴点のマネジメントを行っているはずだが、どうやっている？

#### ソースコードの解析から…

**KeyFrame判定条件**
- First frameはKeyFrame
- max_time_elapsed
- nr_features_low
- stereoFrame_k_->isKeyframe() <=なんかでここ以外で、Keyframe判定される場合がある？
