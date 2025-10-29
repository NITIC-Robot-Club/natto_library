# Natto Library
このライブラリは茨城高専ロボット部でROS 2を使って自動運転やCAN通信などを行う際に便利なパッケージをまとめたものです。

[![auto fix and build full docker](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/autofix_build.yaml/badge.svg)](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/autofix_build.yaml)
[![build docker image](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/docker_build.yaml/badge.svg)](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/docker_build.yaml)
[![Deploy MkDocs](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/mkdocs.yaml/badge.svg)](https://github.com/NITIC-Robot-Club/natto_library/actions/workflows/mkdocs.yaml)

## パッケージ一覧
### common
- [natto_canable](common/natto_canable) : CAN通信を扱う
- [natto_map](common/natto_map) : 地図情報を扱う

### control
- [natto_swerve_calculator](control/natto_swerve_calculator) : N輪独立ステアリングのホイール指令を計算する

### message
- [natto_msgs](message/natto_msgs) : 共通で使うメッセージ定義

### simulation
- [natto_simple_simulator](simulation/natto_simple_simulator) : 簡易シミュレータ

### visualization
- [natto_visualization_converter](visualization/natto_visualization_converter) : RVizで表示するためのトピックに変換する