# natto_speed_path_controller
natto_speed_path_controller パッケージは、SpeedPath に沿って PID 制御でロボットを追従させるノードを提供します。

# speed_path_controller
speed_path_controller ノードは、natto_msgs/msg/SpeedPath を受信し、現在位置との誤差に基づいて PID 制御で速度指令を生成します。

## 機能
- SpeedPath の最近傍点を追従
- 位置（X, Y）とヨー角を独立した PID 制御
- ゴール到達判定と到達通知

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| kp_pos | double | 3.0 | 位置制御の比例ゲイン |
| ki_pos | double | 0.0 | 位置制御の積分ゲイン |
| kd_pos | double | 0.0 | 位置制御の微分ゲイン |
| kp_yaw | double | 3.0 | ヨー制御の比例ゲイン |
| ki_yaw | double | 0.0 | ヨー制御の積分ゲイン |
| kd_yaw | double | 0.0 | ヨー制御の微分ゲイン |
| position_error_allowance_m | double | 0.2 | 修正を行う最大位置誤差（m）。超えると速度0を出力 |
| angle_error_allowance_rad | double | 0.2 | 修正を行う最大角度誤差（rad）。超えると速度0を出力 |
| goal_position_tolerance_m | double | 0.02 | ゴール到達とみなす位置誤差（m） |
| goal_yaw_tolerance_deg | double | 5.0 | ゴール到達とみなすヨー角誤差（度） |
| goal_speed_tolerance_xy_m_s | double | 0.1 | ゴール到達をキャンセルしない最大XY速度（m/s） |
| goal_speed_tolerance_yaw_deg_s | double | 10.0 | ゴール到達をキャンセルしない最大ヨー速度（度/s） |
| frequency | double | 100.0 | 制御ループの周波数（Hz） |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| command_velocity | geometry_msgs/msg/TwistStamped | 計算された速度指令 |
| goal_reached | std_msgs/msg/Bool | ゴール到達状態 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| speed_path | natto_msgs/msg/SpeedPath | 追従するSpeedPath |
| current_pose | geometry_msgs/msg/PoseStamped | 現在の位置と姿勢 |
