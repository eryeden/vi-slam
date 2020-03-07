# vi-slam
Try to make Visual Inertial SLAM


## Docs
- [__Setup__](docs/setup.md)
- [Test environment](docs/about_test.md)
- [Coordinate setup](docs/coordinate_setup.md)
- [Feature detection and tracking](docs/feature_extructor.md)
- [Initialization](docs/initialization.md)
- [Bundle adjustment](docs/ba.md)
- [Dataset](docs/dataset.md)
- [Fig](https://www.draw.io/#Heryeden%2Fvi-slam%2Fmaster%2Fdocs%2Fvislam.drawio)

## Policy
1. Keep a license of this repo as non-GPL.
2. Try to use GTSAM. <= Optimization時に計算が不安定になることがあるらしい。
3. Implement the VETAMIN-E feature.

## Roadmap
![roadmap_march](docs/figs/vislam_development_roadmap.png)

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
    - できればLocalで実行可能なCI toolにしたい
    - werckerはLocalでもいけるらしい
    - Jenkins, Gitlab CI, Droneなどいろいろあるぽい
    - GitLab CIは有料らしいのでJenkinsかDroneがよさそう
    - docker使えるなうな環境としてはDroneがいいか？
- Separate library and utilities
- Write codes and keep style as Google CPP coding style.

## Things need discussions
- Use .cpp or .cc
- Use .hpp or .h

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
- 不利な状況を楽しむようにする
  - 天候：雨、霧、低照度、逆光、直射日光
  - 低FPS、Static仮定を破る環境（カメラの前をウロウロする人がいるとか）、はっきり結像しない環境について（草むら画像をJPEG圧縮した場合など）
  - ライブハウス？レーザー飛び交う現場は一番難しそう。
- https://github.com/PaoPaoRobot/SLAMPaperReading


## Deps
See [__Setup__.](docs/setup.md)


## TODO
### Calibration
- Rollingシャッターのキャリブレーションについて
- 参考:kalibr https://github.com/ethz-asl/kalibr/pull/261
### feature point detection and tracking
- lambdaとsigmaのチューニング
- 同じ点として収束してしまった特徴点の扱い
- 特徴点密度の維持
- 特徴点検出、トラッキング手法の評価方法
### slam
- subspace gauss-newton methodの理解
- Inertial mesurementの導入
- data structure
  - slamの処理に必要なグラフ、データ構造を準備する

## SSHについて
git pullするときのURLを修正する必要あり。
eryedenのリポジトリに対する鍵はgithub-eryedenで紐付けられている。`~/.ssh/config`参照。
なので、`git@github.com:eryeden/vi-slam.git`は`git@github-eryeden:eryeden/vi-slam.git`として設定しなおす必要あり。
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
