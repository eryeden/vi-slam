## BAの実装について

### 詳細
- [式の導出などなど](http://www.iim.cs.tut.ac.jp/~kanatani/papers/budjust.pdf)
- [ヤコビアンによるヘシアンの近似などわかりやすい？](https://ipsj.ixsq.nii.ac.jp/ej/index.php?active_action=repository_view_main_item_detail&page_id=13&block_id=8&item_id=62864&item_no=1)

### 再投影誤差について
ベーシックな方法は、再投影誤差の１次微分g、２次微分Hを活用する。
これが計算できればOK。そのためには、再投影行列についていろいろ感触を調べる必要がある。
ひとまず、5pointのあとに三角測量した点について、再投影行列をもとめて特徴点の再投影差を表示してみたらいいかもしれない。