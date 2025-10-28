# natto_swerve_calculator
natto_swerve_calculator パッケージは、ロボットの指令速度から各ホイールの角度と速度を計算するノードを提供します。

## 機能
- geometry_msgs/msg/TwistStamped メッセージを受信
- 各ホイールの角度と速度を計算
- natto_msgs/msg/Swerve メッセージとしてパブリッシュ

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| infinite_swerve_mode | bool | false | 無限スウェーブモードを有効にするかどうか |
| wheel_radius | double | 0.05 | ホイールの半径 |
| wheel_position_x | std::vector<double> | {0.5, -0.5, -0.5, 0.5} | ホイールのX座標 |
| wheel_position_y | std::vector<double> | {0.5, 0.5, -0.5, -0.5} | ホイールのY座標 |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| swerve_command | natto_msgs/msg/Swerve | 計算されたホイールの角度と速度 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| command_velocity | geometry_msgs/msg/TwistStamped | ロボットの指令速度 |
| swerve_result | natto_msgs/msg/Swerve | ホイールの現在の角度と速度 |