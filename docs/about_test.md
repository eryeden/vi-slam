## テスト環境について
### Github actionsを利用する案
PX4のIssueでこの提案があった。Jenkinsでやるよりもメンテナンスしやすいとか。
プライベートレポジトリの場合、2000分/月のタスクまでただ。
セルフホストされたタスクランナーも利用できるので、これを利用するようにするのがいいかもしれない。ARM環境でも利用できるので、JetsonNanoやXavierでの評価も一緒に実行できるようになると思われる。各アーキテクチャでの処理時間計測などもできるようになると思われる。

参考：
- https://jyn.jp/github-actions-usage/
- https://help.github.com/ja/actions/automating-your-workflow-with-github-actions/about-github-actions
- https://help.github.com/en/actions/automating-your-workflow-with-github-actions/about-self-hosted-runners