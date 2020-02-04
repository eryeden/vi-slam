## BAの実装について

### 詳細
- [式の導出などなど](http://www.iim.cs.tut.ac.jp/~kanatani/papers/budjust.pdf)
- [ヤコビアンによるヘシアンの近似などわかりやすい？](https://ipsj.ixsq.nii.ac.jp/ej/index.php?active_action=repository_view_main_item_detail&page_id=13&block_id=8&item_id=62864&item_no=1)

### 再投影誤差について
ベーシックな方法は、再投影誤差の１次微分g、２次微分Hを活用する。
これが計算できればOK。そのためには、再投影行列についていろいろ感触を調べる必要がある。
ひとまず、5pointのあとに三角測量した点について、再投影行列をもとめて特徴点の再投影差を表示してみたらいいかもしれない。=>表示してみた。間違ってはなさそう。

### BAメモ
一番ベーシックなBA（再投影誤差のみ）でのヤコビアンの計算までは道筋がたった。
しかし、M推定と併用する場合でのGauss-Newton法のやり方が不明。特にロバスト推定用のカーネル関数がBAの再投影コスト関数にかかった場合、どんなヤコビアンを計算すればよいか不明。Ceres-Solverの出力するヤコビアンを利用できれば一番いいだろうが、やり方は不明。M推定を利用するときのGauss-Newton法の解き方については要検討だろう。

### M推定併用のGauss-newtonについて
Ceres-solverのページに方法がのっていた。とりあえず、ヤコビアンさえ求まればM推定カーネルと再投影誤差ベクトルから求まるスケーリング定数をかけるだけでOKぽい。
- [詳細リンク](http://ceres-solver.org/nnls_modeling.html#theory)
- [Ceres-solverの参照先論文](https://hal.inria.fr/inria-00548290/document)

### 逆行列の扱いについて
ヘシアンの逆行列について。基本ヘシアンはJ^T Jで求めるので対称行列になる。部分部分で見た場合は、中心部分の行列は対象になっている。
なので、それらが出現する連立方程式を解く場合はコレスキー分解を利用した方法が使える。Eigenで簡単に計算できるポイので、実装してみる。
Subspace-gauss-newtonを利用するとDenseな話に落とすことができるので、ライセンスの観点からも結構いいかも。BAが収束するまでの全体の計算時間が本法に短くなるのか？がポイントになる？
- [Eigen連立方程式ソルバへのリンク](http://eigen.tuxfamily.org/dox/group__TutorialLinearAlgebra.html)
- [参考サイト](https://ssuzumura.github.io/dev_tips/LeastSquares/solve_LS.html)

### Subspace-Gauss-newton法について
式変形までは納得できるが、計算方法がわからない。パラメータの補正値（δxとか）がほしいのに、一発目の計算から、他のδxを使っていて、どんな値を代入すればいいかわからない。一発目の初期値の計算方法が書かれていないのでどうしようか。
下記論文でも似たような最適化処理を行っているらしいので、δxの初期値について調べてみる必要あり。
- http://vladlen.info/papers/color-mapping.pdf
- https://hal.inria.fr/inria-00548290/document <=有名なやつ

ガウスサイデル法は初期値をすべて0でやっているのがあった？ガウスサイデル法の実装を確認した方が良い。もしかしたら、一旦、ガウスサイデルのようにして、Hδx = -gを解いたあとにxの更新を行うのかもしれない。実際論文もδx_cとδx_pが収束するまで繰り返すといっているので、ほんとにHδx = -gをガウスサイデルで解いて、δxを求めて、ｘの更新をして...みたいなことをやるのかもしれない。しかし、この更新方法は、部分空間Gauss-Newtonなのか？となる…。いづれにせよ、試すパターンは２つ?あとでまとめます。


### 参考文献
- [いろいろ乗っていてありがたい](./paper/Triggs-va99.pdf)


### 3D勉強会のログ

3D勉強会@関東
@3dcvtech

Dec 15, 2019
ORB-SLAM w/o loop closure結構強いんですね
#3dcvtech
3D勉強会@関東
@3dcvtech

Dec 15, 2019
評価方法
- 推定軌跡の平均誤差 [cm]: 精度評価
- 自己位置推定の成功率 [%]: ロバスト性評価
- 初期化のリトライ数 [times]
#3dcvtech
3D勉強会@関東
@3dcvtech

Dec 15, 2019
Euroc datasetはカメラが激しく動き, 露光量の変化も激しいので非常に難しいデータセットだと考えられる
#3dcvtech
3D勉強会@関東
@3dcvtech

Dec 15, 2019
meshのノイズ除去にはNLTGV最小化.
これにTSDFを使うことでメッシュ統合を行っている
(距離の平均値を用いる)
#3dcvtech
3D勉強会@関東
@3dcvtech

Dec 15, 2019
Bundle調整ではこのアプローチは良くないことが知られている
しかし, SLAMの場合には逐次的に計算しているので, 最適化したい変数は最新のフレームのものだけと解釈できる. そのためこの方法が成立していると考えられる
#3dcvtech
3D勉強会@関東
@3dcvtech

Dec 15, 2019
提案手法
- 多次元世紀分布としての解釈
- linear solverとしての解釈
↑変数毎に独立に最適化を行っても, いい感じの解に到達することができる
#3dcvtech
3D勉強会@関東
@3dcvtech

Dec 15, 2019
従来とは異なり, 数万点の最適化の部分に対して, GN法でリアルタイムで解く必要がある
→行ごとに独立に解くことによって解決
#3dcvtech
3D勉強会@関東
@3dcvtech

Dec 15, 2019
アフィン変換ベースのmatchingでは, ある程度のフレームレート(Euroc, KITTI等)でも対応関係を求めることができる感触
#3dcvtech
3D勉強会@関東
@3dcvtech

Dec 15, 2019
[Fine Tracking]
1. 曲率画像kの生成
2. 支配的フローによる予測
3. 山登り法による曲率極大探索
↑フローによって大まかな位置を推定し, 曲率画像上で正確な位置合わせを行う
3D勉強会@関東
@3dcvtech

Dec 15, 2019
[Coarse Matching]
1. 低解像度化
2/ 特徴点抽出&特徴量記述
3. 特徴点対応付け
4. 支配的フロー推定: 画像全体の動きはアフィン変換で近似できるのではないか？
#3dcvtech
3D勉強会@関東
@3dcvtech

Dec 15, 2019
[特徴点追跡]
1. Coarse matching: 特徴点マッチングからアフィン変換を推定
2. Fine Tracking: 特徴量を利用せずに特徴点の対応付け
#3dcvtech
3D勉強会@関東
@3dcvtech

Dec 15, 2019
[VITAMIN-Eまでの実験の道のり]
1. DTAM再現実装
2. VSLAM from Binary images
3. Dense reconst. from sparse points
#3dcvtech
3D勉強会@関東
@3dcvtech

Dec 15, 2019
sparse pointからの復元？
→間引き度合を0.5%などにしてもある程度の密な復元ができることが確認できた
#3dcvtech
3D勉強会@関東
@3dcvtech

Dec 15, 2019
次にバイナリ画像上でtrackingを行えないか？という実装
(それなりな復元が可能に)
→少しでもtrackingが外れると位置推定が困難に
輝度値を使わなくても意外とできる！
#3dcvtech
3D勉強会@関東
@3dcvtech

Dec 15, 2019
形状の復元を密に行いたい(DTAMの再現実装)
→実際に自分で実装すると実環境では非常に難しかった(光の変化へのロバスト性の問題)
#3dcvtech
3D勉強会@関東
@3dcvtech

Dec 15, 2019
特徴点のトラッキングに関しては, 通常のVSLAMと異なり, 画像間のマッチングしか使っていない(mappingの情報を使わない)
→それぞれを並列に実装することができる
#3dcvtech
3D勉強会@関東
@3dcvtech

Dec 15, 2019
feature-baseの問題点: 特徴点を密にマッチングできない
→ 特徴量記述が根本原因 (特徴量記述なしでのトラッキング)
#3dcvtech
3D勉強会@関東
@3dcvtech

Dec 15, 2019
論文はこちらです。
http://openaccess.thecvf.com/content_CVPR_2019/papers/Yokozuka_VITAMIN-E_VIsual_Tracking_and_MappINg_With_Extremely_Dense_Feature_Points_CVPR_2019_paper.pdf
Show this thread
3D勉強会@関東
@3dcvtech

Dec 15, 2019
「denseでfeature-based」であるSLAMは存在しなかったので, その部分を埋めるような提案
#3dcvtech
3D勉強会@関東
@3dcvtech

Dec 15, 2019
[VITAMIN-Eの研究の目標]
1. VSLAMの高精度化及び高ロバスト化
2. 単眼カメラによる実時間三次元環境復元
#3dcvtech
3D勉強会@関東
@3dcvtech

Dec 15, 2019
最初は、産総研の横塚様による
「VITAMIN-E: VIsual Tracking And MappINg with Extremely Dense Feature Points (CVPR2019)」
についての招待講演です。
よろしくお願いいたします。
#3dcvtech
