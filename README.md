# vi-slam
Try to make Visual Inertial SLAM

## Policy
1. Keep a license of this repo as non-GPL.
2. Try to use GTSAM.
3. Implement the VETAMIN-E feature.


## Plan
1. Feature tracker

## What is important?
- Do tests
- Evaluate things by metrics
  - Perfomance metrics
  - Trajectory or Pose metrics
- Use logger
- Use emacs as much as possible
- Use parameter file
- Use CI tools
- Separate library and utilities

## Memo
- 開発指針としたい[資料](docs/problemsandsolutionsforslamdevelopment-191215161142.pdf)
- 評価にデータセットを使うのは大事っぽい
- SLAMは大量のHyperParameterがあるので、これを自動探索できるようになるのは大事っぽい : optuna
- SLAMは密結合なコードとなることが多いので、バグの発見が難しい
  - できるだけ単体テストをする
  - 自動的な統合テストをできるだけ行う
  - 代数計算ツールを活用する
    - SymPyが結構使えそう : [co-lab link](https://colab.research.google.com/drive/1wflhGRVzdlosxHsC63HX2WvXrCG-b8p0)
    - Mathematica使えればベスト
  - テスト結果とパラメータの管理が重要 => mlflow


## SSHについて
git pullするときのURLを修正する必要あり。
eryedenのリポジトリに対する鍵はgithub-eryedenで紐付けられている。`~/.ssh/config`参照。
なので、`git@github.com:eryeden/vi-slam.git`は`git@github-eryeden:eryeden/vi-slam.git`として設定し治す必要あり。
方法は以下の二通り。
1. 設定済みURLをコマンドで修正
``` bash
cd path/to/this/repository
git remote set-url origin git@github-eryeden:eryeden/vi-slam.git
```
2. `git clone`時に修正したURLでCloneする。
``` bash
git clone git@github-eryeden:eryeden/vi-slam.git
```

### 注意
URLである`git@github.com:eryeden/vi-slam.git`の`github-eryeden`の部分は実行するPCの`~/.ssh/config`に依存する。
違う可能性があるので注意。