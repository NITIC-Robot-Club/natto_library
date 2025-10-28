# Natto Library
このライブラリは茨城高専ロボット部でROS 2を使って自動運転やCAN通信などを行う際に便利なパッケージをまとめたものです

[![auto fix and build full docker](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/autofix_build.yaml/badge.svg)](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/autofix_build.yaml)
[![build docker image](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/docker_build.yaml/badge.svg)](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/docker_build.yaml)
[![Deploy MkDocs](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/mkdocs.yaml/badge.svg)](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/mkdocs.yaml)

## [ドキュメントはこちら](https://nitic-robot-club.github.io/natto_library/)

## 目次
- [インストール方法](#インストール方法)
- [追加予定](#追加予定)

## インストール方法
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

## 追加予定
- natto_lidar_merger : 複数のLiDARデータを統合するパッケージ
- natto_pointcloud2_utils : PointCloud2を扱うユーティリティ
- natto_ransac : RANSACアルゴリズムを実装したパッケージ
- natto_wheel_odometry : 車輪オドメトリを計算するパッケージ
- natto_mcl : Monte Carlo Localizationを実装したパッケージ
- natto_ekf_localizer : 拡張カルマンフィルタを用いた自己位置推定パッケージ
- natto_astar_planner : A*アルゴリズムを用いた経路計画パッケージ
- natto_speed_planner : 速度計画を行うパッケージ
- natto_pure_pursuit : Pure Pursuitアルゴリズムを用いた追従制御パッケージ
- natto_pid_controller : PID制御で速度追従を行うパッケージ
- natto_map_visualizer : 地図データを可視化するパッケージ
- natto_map_utils : 地図データを扱うユーティリティ