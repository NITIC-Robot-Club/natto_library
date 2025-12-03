# natto_mcl
natto_mcl パッケージは、モンテカルロ局所化（MCL）アルゴリズムを使用して、ロボットの自己位置推定を行うノードを提供します。

## 機能
- レーザースキャンデータと地図情報に基づく自己位置推定
- パーティクルフィルタを使用した高精度な位置推定
- ロボットの動作モデルとセンサーモデルの統合


## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| map_frame_id | string | "map" | 地図フレームのID |
| odom_frame_id | string | "odom" | オドメトリフレームのID |
| base_frame_id | string | "base_link" | ベースフレームのID |
| num_particles | int | 500 | 使用するパーティクルの数 |
| initial_pose_x | double | 0.0 | 初期位置のX座標 |
| initial_pose_y | double | 0.0 | 初期位置のY座標 |
| initial_pose_theta | double | 0.0 | 初期姿勢の角度（ラジアン） |
| motion_noise_xx | double | 0.02 | | 動作モデルのノイズ（X方向） |
| motion_noise_xy | double | 0.0 | 動作モデルのノイズ（XY方向） |
| motion_noise_yy | double | 0.02 | 動作モデルのノイズ（Y方向） |
| motion_noise_theta | double | 0.01 | 動作モデルのノイズ（角度） |
| motion_noise_position | double | 0.01 | 位置ノイズの大きさ |
| motion_noise_orientation | double | 0.01 | 姿勢ノイズの大きさ |
| expansion_radius_position | double | 1.0 | 位置の拡張半径 |
| expansion_radius_orientation | double | 3.14 | 姿勢の拡張半径 |
| laser_likelihood_max_dist | double | 0.2 | レーザーセンサの尤度計算における最大距離 |
| transform_tolerance | double | 0.2 | 変換の許容時間 |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| pose | geometry_msgs/msg/PoseStamped | 推定されたロボットの位置 |
| particles | geometry_msgs/msg/PoseArray | パーティクルの位置 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| occupancy_grid | nav_msgs/msg/OccupancyGrid | 地図情報 |
| pointcloud2 | sensor_msgs/msg/PointCloud2 | レーザースキャンデータ |
| initial_pose | geometry_msgs/msg/PoseWithCovarianceStamped | 初期位置の設定 |