# 開発者ガイド
## インストール
このプロジェクトをローカル環境にセットアップするには、以下の手順に従ってください
1. リポジトリをクローンします
   ```bash
   git clone git@github.com:NITIC-Robot-Club/natto_library.git
   ```

2. ワークスペースをビルドします
   ```bash
   cd natto_library
   colcon build --symlink-install
   ```

3. 環境をセットアップします
   ```bash
   source install/setup.bash
   ```
   ~/.bashrcに追加しておくと便利です
   ```bash
   echo "source ~/natto_library/install/setup.bash" >> ~/.bashrc
   ```

## コードスタイル
C++コードは、[Google C++ Style Guide](https://google.github.io/styleguide/cppguide.html)をベースとしています
コミット時にautofixで自動整形が行われます

VSCodeを使用している場合はC++の拡張機能をインストールするとローカルでかんたんにフォーマットできて便利です

またこのライブラリは最低限の自動運転を行うのに必要な依存関係を最小限に抑えるため以下のような外部ライブラリの使用は原則認められません
- Boost
- Eigen
- OpenCV
- PCL

ただし以下のようなケースは例外とします
- AI(広義)を使い画像から物体検出など外部ライブラリがないと話にならないが自動運転を行うのに必須ではないパッケージ開発

上記の例外に該当する場合はIssueやDM等で相談後に開発を開始してください

また、例外的に使用が許可される外部ライブラリについては、インストール手順を適切にドキュメントに記載し、Dockerfileにも反映させる必要があります

外部ライブラリを使用する場合は、ライセンスがプロジェクトの要件と互換性があることを確認してください

|| `sudo apt install`+α だけでインストールが終わるものが好ましいです ||


## 開発フロー
1. 新しい機能やバグ修正のためにブランチを作成します
   ```bash
   git checkout -b feat/your-feature-name
   ```
2. 変更をコミットしpushします
3. PRを作成し、ビルドが通ることを確認後、レビューを依頼します
4. レビューが完了し、承認されたらmainブランチにマージされます
