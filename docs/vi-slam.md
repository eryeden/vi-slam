# Kimera-VIO

## VIO Front-end
- 特徴点：Shi-Tomasi corners
- トラッキング：Lukas-Kanade tracker
- Verification?特徴点の誤マッチ？の検出
  - mono: 5-point ransac。
  - stereo: 3-point ransac。
  - IMU併用で、1-point（ステレオ）か、2-point（Mono） ransacを利用する 
- Key frameでの処理: 
  - feature detection
  - geometric verification
- Intermediate frameでの処理:
  - Feature trackingのみ
- IMUは積分する

## VIO Back-end
- Key frameに対してBA?を実施する
- Fixed-lag smootherなので固定WindowサイズでBAをやる感じなのかな？
- structureless vision model ???
- Factor graphは、GTSAMのiSAM2を使って解くらしい
- iSAM2での処理について
  - それぞれの最適化ステップで、DLTを使って観測したFeatureの３D positionを計算する、DLTで初期値設定されたものと言う意味？
  - VIO状態から、解析的に、関係する３D pointを除去する？？？？？
  - 除去に際して：
    - pointのdegenerateを行う；カメラの裏のポイントや、Triangulation時に十分な視差が内場合など
   - Outlierが除去される；Reprojection errorが大きいポイントは除去する
   - これによって、Extra robustness layerになるとある
   - Robustness layerとは？？？
   - Smoothing horizonに落っこちた様態が、merginalized outされる。

## わかったこと
- KimeraもKey frameベース
- Key frameでは、毎回5点法＋RANSACでOutlierを除去している
- Key frameでないFrameはFeature pointの追跡のみやる
- トラッキングは Lukas-Kanade
- 特徴点はShi-Tomasi corner
- Kimera-VIOは基本的に`On-Manifold Preintegration for Real-Time Visual-Inertial Odometry`この論文の実装になっているぽい？

## わからないこと
- Keyframeの選択方法
- Merginalize outの具体的意味
- PointのDegenerateとは？
- iSAM2のFixed-lag smootherでの処理内容


## `On-Manifold Preintegration for Real-Time Visual-Inertial Odometry`の内容を見てみることにする
多分ここにいろいろ書いてあると思う。Loop closerなしで結構の精度を出しているので、参考になることは多そう。
わからないことを注目点としてすすめる。

- 問題意識：時間がたってTrajectoryのが長くなると、Real−timeのoptimizationがすぐにできなくなってしまう。この問題は、IMUの観測レートが高く、最適化対象の変数数が早く増えてしまうことからも大きな問題になってしまっている。

- なので：選択したKeyFrame間のIMU観測値を事前に積分して、一つの相対的なMotion拘束条件に変換することで、「最適化対象の変数急増問題」に対処している。


- 論文の売り１：Preintegrationのやり方提案
  - Rotation群（Lie群）のManifold structureを正しく扱えるらしい
  - なにをやっているかというと、IMU回転のRotation誤差を含めるような観測値生成モデルについて検討して、事後生起確率？を最大化する状態推定器を導いている。
  - これによって、解析的にヤコビアンと事後バイアス補正量？の計算ができるようになる。
- 論文の売り２：このPrinegration IMUモデルが、シームレスにVIOパイプラインに、Factor graphのフレームワークで組み込むことができることを示したこと。
  - これによって、`incremental-smoothing algorithms`をやれるし、
  - `structureless model`というもの扱えるようになる；全Feature pointを使う必要はなくなる？

### Intro読んでみて
IMUをFactor graph構造に組み込む方法が論文のメインになっているように見える。
IMUはひとまずおいておく、これ以外の部分について重点的に読み込む。


### わからないこと
- IMU系
  - Lie群？のManihold structureとは？
  - Incremental smoothing algorithmとは？
  - Structure less modelとは？




