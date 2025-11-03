# Natto Library
このライブラリは茨城高専ロボット部でROS 2を使って自動運転やCAN通信などを行う際に便利なパッケージをまとめたものです

[![auto fix and build full docker main](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/autofix_build_main.yaml/badge.svg)](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/autofix_build_main.yaml)
[![build docker image](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/docker_build.yaml/badge.svg)](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/docker_build.yaml)
[![Deploy Docs and Map Builder](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/mkdocs.yaml/badge.svg)](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/mkdocs.yaml)

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
- natto_wheel_odometry : 車輪オドメトリを計算するパッケージ
- natto_mcl : Monte Carlo Localizationを実装したパッケージ
- natto_ekf_localizer : 拡張カルマンフィルタを用いた自己位置推定パッケージ
- natto_speed_planner : 速度計画を行うパッケージ
- natto_pid_controller : PID制御で速度追従を行うパッケージ
