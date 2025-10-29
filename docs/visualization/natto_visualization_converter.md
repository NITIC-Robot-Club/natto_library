# natto_visualization_converter
natto_visualization_converter パッケージは、natto_msgs を可視化用のメッセージに変換してパブリッシュするノードを提供します。

# map_visualizer
map_visualizer ノードは、natto_msgs/msg/Map メッセージを受信し、可視化用の MarkerArray メッセージに変換してパブリッシュします。

## 機能
- natto_msgs/msg/Map メッセージを受信
- 直線セグメントと円弧情報を可視化用の Marker メッセージに変換
- visualization_msgs/msg/MarkerArray メッセージとしてパブリッシュ

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| publish_period_ms | int | 1000 | マーカー配列をパブリッシュする周期（ミリ秒） |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| marker_array | visualization_msgs/msg/MarkerArray | 変換した地図情報のマーカー配列 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| map | natto_msgs/msg/Map | 地図情報 |

# swerve_visualizer
swerve_visualizer ノードは、natto_msgs/msg/Swerve メッセージを受信し、可視化用の Marker メッセージに変換してパブリッシュします。

## 機能
- natto_msgs/msg/Swerve メッセージを受信
- ホイールの角度と速度情報を可視化用の Marker メッセージに変換
- visualization_msgs/msg/MarkerArray メッセージとしてパブリッシュ

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| publish_period_ms | int | 10 | マーカー配列をパブリッシュする周期（ミリ秒） |
| arrow_r | double | 1.0 | 矢印マーカーの赤成分（0.0〜1.0） |
| arrow_g | double | 1.0 | 矢印マーカーの緑成分（0.0〜1.0） |
| arrow_b | double | 0.0 | 矢印マーカーの青成分（0.0〜1.0） |
| arrow_scale | double | 0.2 | 矢印マーカーのスケール係数 |
| arrow_min_size | double | 0.1 | 矢印マーカーの最小サイズ |
| wheel_position_x | double[] | [0.5, -0.5, -0.5, 0.5] | 各ホイールのX座標位置 |
| wheel_position_y | double[] | [0.5, 0.5, -0.5, -0.5] | 各ホイールのY座標位置 |


## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| marker_array | visualization_msgs/msg/MarkerArray | 変換したホイール情報のマーカー配列 |


## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| swerve | natto_msgs/msg/Swerve | ホイールの角度と速度情報 |