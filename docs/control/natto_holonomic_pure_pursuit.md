# natto_holonomic_pure_pursuit
natto_holonomic_pure_pursuit パッケージは、ホロノミックロボットのためのピュアパシュート制御ノードを提供します。

## 機能
- 全方位移動ロボットのためのピュアパシュート制御
- 目標経路と現在位置に基づいて指令速度を計算
- 加速度、減速度、最高速度、最低速度の制約を考慮

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| control_frequency | double | 50.0 | 制御ループの周波数（Hz） |
| lookahead_time_s | double | 1.0 | 先読み時間（秒） |
| min_lookahead_distance_m | double | 0.1 | 最小先読み距離（メートル） |
| max_lookahead_distance_m | double | 1.0 | 最大先読み距離（メートル） |
| yaw_speed_p | double | 1.0 | ヨー速度比例ゲイン |
| curvature_decceleration_p | double | 1.0 | 曲率減速度比例ゲイン |
| min_curvature_speed_m_s | double | 0.3 | 曲率に基づく最低速度（メートル毎秒） |
| yaw_decceleration_p | double | 1.0 | ヨー減速度比例ゲイン |
| max_speed_xy_m_s | double | 3.0 | XY方向の最高速度（メートル毎秒） |
| min_speed_xy_m_s | double | 0.1 | XY方向の最低速度（メートル毎秒） |
| max_speed_yaw_deg_s | double | 180.0 | ヨー方向の最高速度（度毎秒） |
| min_speed_yaw_deg_s | double | 18.0 | ヨー方向の最低速度（度毎秒） |
| max_acceleration_xy_m_s2_ | double | 10.0 | XY方向の最高加速度（メートル毎秒毎秒） |
| max_acceleration_yaw_deg_s2 | double | 500.0 | ヨー方向の最高加速度（度毎秒毎秒） |
| goal_deceleration_m_s2 | double | 4.0 | 目標減速度（メートル毎秒毎秒） |
| goal_deceleration_distance_p | double | 1.0 | 目標減速度距離比例ゲイン |
| goal_position_tolerance_m | double | 0.03 | 目標位置許容誤差（メートル） |
| goal_yaw_tolerance_deg | double | 10.0 | 目標ヨー許容誤差（度） |
| goal_speed_tolerance_xy_m_s | double | 0.3 | 目標速度許容誤差XY方向（メートル毎秒） |
| goal_speed_tolerance_yaw_deg_s | double | 30.0 | 目標速度許容誤差ヨー方向（度毎秒） |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| command_velocity | geometry_msgs/msg/TwistStamped | 計算された指令速度 |
| lookahead | geometry_msgs/msg/PoseStamped | 先読み位置 |
| goal_reached | std_msgs/msg/Bool | 目標到達状態 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| current_pose | geometry_msgs/msg/PoseStamped | ロボットの現在位置 |
| path | nav_msgs/msg/Path | 目標経路 |
