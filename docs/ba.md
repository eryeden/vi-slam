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