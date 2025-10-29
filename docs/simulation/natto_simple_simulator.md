# natto_simple_simulator
簡易シミュレータはロボットの基本的な動作を模擬します

# swerve_simulator
N輪独立ステアリングロボットの動作を模擬します。

## 機能
- ホイールの速度とステアリング角度からロボットの移動を計算します。
- ロボットの位置と姿勢を更新します。

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| infinite_swerve_mode | bool | false | 無限ステアリングモードを有効にします |
| wheel_radius | double | 0.05 | ホイールの半径（m） |
| wheel_position_x | std::vector<double> | {0.5, -0.5, -0.5, 0.5} | 各ホイールのx座標（m） |
| wheel_position_y | std::vector<double> | {0.5, 0.5, -0.5, -0.5} | 各ホイールのy座標（m） |
| angle_gain_p | double | 100.0 | ステアリング角度の比例ゲイン |
| angle_gain_d | double | 0.0 | ステアリング角度の微分ゲイン |
| speed_gain_p | double | 100.0 | ホイール速度の比例ゲイン |
| speed_gain_d | double | 0.0 | ホイール速度の微分ゲイン |
| simulation_period_ms | int | 1 | シミュレーションの周期（ms） |
| initial_pose_x | double | 1.0 | 初期位置のx座標（m） |
| initial_pose_y | double | 1.0 | 初期位置のy座標（m） |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| /swerve_result | natto_msgs::msg::Swerve | シミュレートされたホイールの角度と速度 |
| /pose | geometry_msgs::msg::PoseStamped | ロボットの現在の位置と姿勢 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| /swerve_command | natto_msgs::msg::Swerve | ホイールの目標角度と速度 |
