# natto_joy
joyパッケージからのジョイスティック入力を受け取り、ロボットの移動コマンドに変換してパブリッシュするノードを提供します。

# joy_to_cmd_vel
joy_to_cmd_velノードは、ジョイスティックの入力を受け取り、ロボットの移動コマンド（geometry_msgs/msg/Twist型）に変換してパブリッシュします。

## 機能
- sensor_msgs/msg/Joy型のジョイスティック入力を受信
- geometry_msgs/msg/TwistStamped型の移動コマンドをパブリッシュ

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| max_xy_speed_m_s | double | 2.0 | 前後・左右移動の最大速度 (m/s) |
| max_theta_speed_rad_s | double | 3.1415 | 回転の最大速度 (rad/s) |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| command_velocity | geometry_msgs/msg/TwistStamped | ロボットの移動コマンド |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| joy | sensor_msgs/msg/Joy | ジョイスティックの入力データ |