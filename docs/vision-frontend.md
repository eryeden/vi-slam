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
- まえのKeyFrameから一定時間断っている
- トラッキングしている特徴点が一定以下になる

**特徴点**
- 特徴点検出は、Keyframeのみ。RANSACでOutlier除去を行った後で特徴点検出する。
- Shi-Tomasi cornersを検出して、OpenCVの`calcOpticalFlowPyrLK`で特徴点と追跡
- トラッキング特徴点数でKeyFrameにするかしないかの判断をするが、この判断の内容は？
  - calcOpticalFlowPyrLKのStatusが０でない（特徴点のフローが検出されていない状態を示す）
  - Landmark ageがmax feature age以下、feature ageは何回、連続したKeyFrameに観測されたか？という指標
    - なので、Feature trackerの関数ではfeature ageは増えない

**Outlier除去**
- Front endで実施されるOutlier除去はGeometric verificationと呼ばれている
- 一応、Trackerの内部でも、LKで追跡に成功したもの、失敗したのもなど検出しており、質の悪いTrackingが発生しないようになっている
- Geometric verification
  - Tracker::geometricOutlierRejectionMonoが相当
  - 入力：２Frameのみ
  - 処理：５Point RANSACで誤対応の特徴点を削除、Inlier数と視差中央値のチェック結果を出力する。
    1. 両フレームで共通するLandmark idを選択する。
    2. それぞれの特徴点の観測FrameにおけるBearingAngleをゲット。
    3. OpenGVの5pointRANSACを計算。解が見つからないときはINVALIDでReturnする。
    4. RANSAC結果に基づいてOutlierの除去を行う。
    5. Trackingのクオリティチェック。Inlierの数が基準以下ならばStatusとしてFEW_MATCHESを出力。
    6. 視差チェック。全特徴点のDisparityの中央値を計算をして小さいならばLOW_DISPARITYをStatusに追加。

