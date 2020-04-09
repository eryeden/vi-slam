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

```
C. Forster, L. Carlone, F. Dellaert, and D. Scaramuzza, “On-manifold
preintegration theory for fast and accurate visual-inertial navigation,”
IEEE Trans. Robotics, vol. 33, no. 1, pp. 1–21, 2017, arxiv preprint:
1512.02363, (pdf), technical report GT-IRIM-CP&R-2015-001
```


### Abst

- 問題意識：時間がたってTrajectoryのが長くなると、Real−timeのoptimizationがすぐにできなくなってしまう。この問題は、IMUの観測レートが高く、最適化対象の変数数が早く増えてしまうことからも大きな問題になってしまっている。

- なので：選択したKeyFrame間のIMU観測値を事前に積分して、一つの相対的なMotion拘束条件に変換することで、「最適化対象の変数急増問題」に対処している。


- 論文の売り１：Preintegrationのやり方提案
  - Rotation群（Lie群）のManifold structureを正しく扱えるらしい
  - なにをやっているかというと、IMU回転のRotation誤差を含めるような観測値生成モデルについて検討して、事後生起確率？を最大化する状態推定器を導いている。
  - これによって、解析的にヤコビアンと事後バイアス補正量？の計算ができるようになる。
- 論文の売り２：このPrinegration IMUモデルが、シームレスにVIOパイプラインに、Factor graphのフレームワークで組み込むことができることを示したこと。
  - これによって、`incremental-smoothing algorithms`をやれるし、
  - `structureless model`というもの扱えるようになる；全Feature pointを使う必要はなくなる？し計算時間が短くなる。

**Abst読んでみて**
IMUをFactor graph構造に組み込む方法が論文のメインになっているように見える。
IMUはひとまずおいておく、これ以外の部分について重点的に読み込む。

### Intro
いままであった、精度と計算速度のトレードオフを打ち破る！！

- IMUによってMetric を復元できる
- いままでのVIOには、精度と計算コストのトレードオフがあった
- ２つの流派がある。
  - Filtering approache：計算は軽いけど、計算時の非線形性によって性能が悪化する可能性あり。
  - Full smoothing approache：正確だけど計算コストが大きい。Fixed-lag smoothingを利用することで精度と計算コスト間の調整が可能だが、要求される精度・計算コストを達成できるWindowサイズをどう決めればよいのかは、よくわかっていない。
- この論文のキモとして、このトレードオフを打ち破ることができたこと、が挙げられる
  - 高速にIncremantal smoothingをやって、最適なMAPをリアルタイムで推定するらしい

- やったこと
  - IMUの事前積分について理論を開発
    - Keyframe間のIMU観測情報を積分して、一つのMotion拘束条件として利用する話は前からあって、これに乗っかる形
    - SO3の回転群のManifold structureを正しく扱えるPreintegration理論を考えた。
    - Rotation noiseの扱いが厳密になっている
    - 回転表現の特異点が回避できている
    - 必要なすべてのJacobianやNoise propagation、事後バイアス補正についての式が全て書いてある！付録に。
    - ※2012年くらいの研究からVision + IMUの研究がスタートしている？この論文は2017パブリッシュ。
  - Factor graph modelにIMU-preintegrationについて導入したこと
    - これによって、incremental smoothingを使った手法が実装できている：iSAM2
    - 線形化したときの誤差が累積していく？ことを避けられている
    - 精度と計算コストをいい感じの方法で調節できる
    - Visual mesurementにおいて`Struture less model`を導入
      - Incremental smoothingにおいて、大量の最適化対象変数をなくすことができる
      - 計算の高速化につながる
- `Structure less model`
  - Incremental smoothingというVIOのフレームワークを使っている。
    - 比較対象は、MSCKF(`Multi-State Constraint Kalman Filter`)らしい。拘束条件を扱えるカルマンフィルタ？
  - 最適化windowをずらしながら逐次的に最適化をかけていく方法？
  - ２つのいいところがある
    - Visual mesurement（特徴点の検出、トラッキングの話？）の処理完了を待たなくて良い
    - Visual mesurementの再線形化を何度も行うことができる（カルマンフィルタだと過去の状態をすべて保存していないから？）


- 実験内容
  - RealとSimulationのデータセット両方で検証した
  - いいところが結構あったよ
    - 論文のアプローチを実装すれば、Full-smoothingを100Hzで実行できる
    - 競合のVIOと比べて（optimization baseとfiltering base両方とも）いい精度を出せる
    - この論文は実装するひとにとっても良くて、チュートリアルにもなりうる
      - 論文では、`Uncertainty representation`について、短くて完結な表現がなされている。
      - 再現実装するのに必要で、全部知りたい場合は、付録を参照してくれ。↓の内容が乗っているらしい。
        - Uncertainty representation on manifolds
        - Examplary derivations for uncertainty propagation（確率的な事象の伝搬？の代表的な導き方）
        - Jacobian computation

- 論文の内容
  - まえの論文の拡張らしい
    - `IMU Preintegration on Manifold for Efficient Visual-Inertial Maximum-a-Posteriori Estimation`
    - 全体的な手法の導き方など追加があるそうなので、前は追わずにこのまま読み進めたほうがいい気がする
  - 追加内容
    - Bias estimationの精度評価実験
    - この手法のConsistencyのデモ
    - Full batch estimationとの比（fixed-lag sommtherではなく、データ全体を使った計算との差？）
  - GTSAM4.0にここでの内容は追加されているよ

- ちょい目を通したが・・・
  - この論文はIMU preintegrationをいかに導入するか？という内容が多い
  - しかし、Strecture less modelというものを利用しているらしく、これは調べてみたい。


### Related work

３つの方向性からVIOの研究の方向性を見ることができるらしい。以下の３つ。
1. カメラPoseの数が増えて、この過去の観測情報をどうやって扱うか？
   1. Full smootherやBatch nonlinear 最小二乗法では、昔のPoseをすべて推定する
   2. Fixed-lag smoother（Sliding window）では、Window内の最新Poseのみ推定
   3. Filtering approachesは、最新のPoseのみを推定する
   - Fixed-lagとFilteringは基本、最適化しない昔のパラメータはMarginalize
   - 関係性？？のパラメータはGaussian priorとしてAbsorbするらしい
2. いかに、観測値や、Gaussian prior（事前分布？？？）の不確実性を表現するか？
   1. EKF:不確実性は共分散で表現
   2. 情報Filter、Smootherは、情報行列を利用（共分散行列の逆数）、もしくは情報行列の平方根？を利用
3. 何回観測モデルを線形化するか？
   1. StandardなEKFは一回の観測に対して一回の処理しかしない。一つの状態ベクトルと共分散行列に一回の観測が集約されてしまうということ？
   2. Smoothing approachは何回もLiniearizingできる。一つに観測値に対して何回も最適化をかけることができるという話か！？

用語としてはいろいろあるが、背後にあるアルゴリズムは密接に結びついているらしい。
実際、`Iterated Extended Kalman filter`の式はGauss-Newtonの更新式？と同じになるらしい。


以下、３種類の過去情報の扱い方についてまとめる。
#### A. Filtering approach
Filtering baseの手法では、推定対象の処理をシステムの状態の最新のものに限定することで、低計算コストな推定を行えている。
EKFの計算複雑性は、観測Landmark数にたいして２次的に増加する。なので、リアルタイム処理をしたいシステムでは、一般的にトラックする特徴点数は少なくなっている。だいたい２０個くらい。

この問題に対しての回避策として、`structureless`approachがある。

*`Structureless`approachとは？*
Landmark positionというのがEKF?の状態ベクトルからmarginalizeされて消える？
いままでのFilterベースの方法では、Landmarkの数が増えると二乗で計算コストが増えてしまうのが問題だった。
StructurelessではLandmark部分がMarginalize outされるので計算コスト増加がカメラPoseが増えることによるものだけになる？これがいいところ？

*`Structureless approach`の欠点*
- Filteringベースの方法で使うためには、Landmarkをすべて観測？？？するまで、Structurelessの計算を実行できないこと。これが原因で、すべてのVisual informationを利用できないので精度が落ちてしまうことがある。
- Marginalizationが、線形化誤差とOutlierの観測を固定化？してしまうこと。これも誤差の要因になりうる。
- 一つのOutlierがFilterを回復不能にしてしまう可能性があるので、怪しい観測値を省くのが重要になるらしい。
- 線形化誤差：ドリフトとFilterのInconsistentが発生する
- Inconsistencyの影響：Over-confidentが発生しうる。これによって、最適でないInformation fusionが起きるらしい。どれかの共分散がかなり大きくなってしまってフィルターが発散したりするとか？

*VIOでの直接観測できない情報*
- 基本４つある
- 1~3で、global position
- 4つ目で、重力方向の回転角
- このうち、間違った推定が起きるときの線形化では、Global positionの値しか推定することができない、という報告もあるので、間違っている線形化では、間違ったYawの情報がGaussian priorに加えられてしまう。これによってFilter inconsistentが起きてしまう。
- この問題は、`first-estimates jacobian`アプローチで対策が取られている。このアプローチで、Inconsistencyの原因となる間違った線形化を行っている特徴点を使ってFilterの状態が更新されないことを保証できる。
- `Observability-constrained EKF`では、可観測の情報のみがフィルタを更新できるようにすることで、直接観測できない状態両の観測を行う。
- VIOの可観測性についてはいろいろと研究されているらしい。


**まとめると**
- 普通のFilter approach
  - 特徴：Landmark数が増えると計算コストが大きくなってしまうのが問題
- Structureless approach
  - いいところ：Landmark部分がMarginalizeされるのでカメラPoseの増加分のみ計算コストが増大する？
  - 欠点：計算の遅延、線形化誤差、ハズレ値の観測、可観測でない情報の扱い、これによって生じるFilter inconsistency
  - 対応１：`first-estimates jacobian`
  - 対応２：`Observability-constrained EKF`



#### B. Fixed-lag Smoothing approach
基本、Fixed-lag smootherは、Time window内にある状態を推定して、それ以外の古い状態はMarginalize outする。
Maximum likehood estimationでは、範囲内の最近の状態における最適化問題に集約している。
基本、Smoothing aproacheはFilterベースの手法よりも高精度になる。なぜかというと、過去の状態を何回も線形化できるから。
**いいところ** : Filter baseの手法よりもOutlierに強い。最適化後にOutlierを消すこともできるし、Robust cost functionで影響を軽減することができるため。
**わるいところ** : 推定Time windowの外にある状態はMarginalizeするのたけど、ここで作られるGaussian priorが粗行列にならない場合がある。これが推定の計算コストを上げてしまう可能性あり。このために、粗行列になるような観測情報を消す方法など提案されている。また、Fixed-lag smootherにおいてもFiltering approachでの問題も存在する。Consistencyや、線形化誤差が溜まっていくことなど。


#### C. Full Smoothing approach
基本、大規模な非線形最適化問題を解くことで、過去全ての状態（カメラの状態と特徴点位置すべて）を推定する。
精度は一番いいが、Realtime処理はマップやカメラ軌跡が増えるに従って、すぐに困難になる。
**この対策** : Keyframeを除いて、ほかの情報を捨てる、最適化を別スレッドで走らせるTrackingとMappingを両方やるDual archtectureなど。

**ここでBreakthroughがあった！！！！** : `Incremental smoothing techniques`の登場である。iSAM、iSAM2ではこれを使っているらしい。具体的には、Factor graphを使っている。これを使うことによって、行列Sparsityをキープできる。加えて、新しく観測した情報に影響されうる小規模なサブセットの状態のみ更新することができる。

**Breakthroughがあったが、以前、更新頻度の高いIMUを扱うには高いハードルがある** : 
(最適化が必要な状態の数がすぐに増えてしまうため？)
何個か実装の種類がある。
1. naive implementation : IMUのすべての観測情報をすべて追加する。この実装は実用的でないくらいに遅くなってしまう。
2. ２フレーム間のIMU観測情報を積分して、２フレーム間の相対的な移動量の拘束条件にする：これにも問題があって、IMUを積分するときの初期状態が一番最初に推定されたFrameの状態によって影響される。しかし、最適化の仮定で、Frameの推定値が変化するとすべてのIMU積分をやり直す必要が生まれてしまう。
3. IMU preintegration : 2の方法で必要なIMU積分のやり直しは、相対的な動きの拘束条件を`Reparametrization`することで回避できて、計算量を削減できる。これが、IMU preintegrationをと呼ばれるもの。

### この論文でやっていること
1. IMU preintegrationの厳密な導出：Preintegrationを初めて導入した論文があるのだけど、オイラー角を使っている問題がある。オイラー角を、状態推定、共分散推定のための、AveraginやSmoothing手法に利用すると、Rigid transformationにおいて不変（性質が不変？？）ではない場合がでてくる。あと、オイラー角には特異点もある。この論文えは、回転の観測に厳密な扱いをして最尤推定着の完全な式の導出を行っている。最適化の計算に必要なJacobianの解析的な式も載せている。

### 結果どうだった？
Rotation manifoldを厳密に扱うことで、基の論文に比べて、高い精度とロバスト性があることが確認できている。


### `Structureless approach`についてのみ調べてみる
- iSAMでの実装について
- `Incremental smoothing techniques`の導入について
- そもそもVisionだけで`Structure less`を計算できるのか？ 



### わからないこと
- IMU系
  - Lie群？のManihold structureとは？
  - Incremental smoothing algorithmとは？
  - Structure less modelとは？
- Vision系
  - この論文のVision frontendはSVOがベースになっている。SVOではFeatureの追跡時にDepthを利用することでエピポーラの拘束を満たしたFeatureのみ追跡できるという利点がある。
  - しかし、Kimera-VIOでは特徴点検出がShi-Tomasi corners、トラッキングがLK-tracker
  - ここの違いはどうなっているのか？
- カメラPoseの話は結構でてくるが、Feature pointやMapの話はでてくるのか？







## 単語
| 単語      | 意味                                               |
| --------- | -------------------------------------------------- |
| tangible  | 実感できる、これええやんって実感できる？           |
| exemplary | 代表的な、これやっときゃとりあえずOkみたいな感じ？ |
| lead to   | ~になるように導く                                  |
