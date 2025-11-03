# Natto Library
このライブラリは茨城高専ロボット部でROS 2を使って自動運転やCAN通信などを行う際に便利なパッケージをまとめたものです。

[![auto fix and build full docker main](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/autofix_build_main.yaml/badge.svg)](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/autofix_build_main.yaml)
[![build docker image](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/docker_build.yaml/badge.svg)](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/docker_build.yaml)
[![Deploy Docs and Map Builder](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/mkdocs.yaml/badge.svg)](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/mkdocs.yaml)

## パッケージ一覧
### common
- [natto_canable](common/natto_canable) : CAN通信を扱う
- [natto_map](common/natto_map) : 地図情報を扱う

### control
- [natto_holonomic_pure_pursuit](control/natto_holonomic_pure_pursuit) : ホロノミックロボットのためのピュアパシュート制御を行う
- [natto_swerve_calculator](control/natto_swerve_calculator) : N輪独立ステアリングのホイール指令を計算する

### message
- [natto_msgs](message/natto_msgs) : 共通で使うメッセージ定義

## planning
- [natto_astar_planner](planning/natto_astar_planner) : A*アルゴリズムを用いた経路計画を行う

### sensing
- [natto_laser_filter](sensing/natto_laser_filter) : LiDARのレーザースキャンデータをフィルタリングする
- [natto_lidar_converter](sensing/natto_lidar_converter) : LiDARのデータ変換する
- [natto_lidar_merger](sensing/natto_lidar_merger) : 複数のLiDARデータをマージする
- [natto_ransac](sensing/natto_ransac) : RANSACアルゴリズムを用いて点群データから形状を検出する

### simulation
- [natto_simple_simulator](simulation/natto_simple_simulator) : 簡易シミュレータ

### visualization
- [natto_visualization_converter](visualization/natto_visualization_converter) : RVizで表示するためのトピックに変換する


## Map Builder 
マップを作成するためのwebベースのツール

[Map Builder URL](map_builder){: target="_blank"} (クリックすると別のタブが開きます)
