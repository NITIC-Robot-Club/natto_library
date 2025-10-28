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

VSCodeを使用している場合はC++の拡張機能をインストールすると

## 開発フロー
1. 新しい機能やバグ修正のためにブランチを作成します
   ```bash
   git checkout -b feat/your-feature-name
   ```
2. 変更をコミットしpushします
3. PRを作成し、ビルドが通ることを確認後、レビューを依頼します
4. レビューが完了し、承認されたらmainブランチにマージされます
