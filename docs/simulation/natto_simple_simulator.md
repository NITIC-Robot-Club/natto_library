# natto_simple_simulator
簡易シミュレータはロボットの基本的な動作を模擬します

# joint_state_simulator
ジョイントステートシミュレータはロボットのジョイント状態をシミュレートし、ロボットの位置と姿勢を計算します。

## 機能
- コマンドジョイント状態を受信し、シミュレートされたジョイント状態を生成します。
- ホイールの速度とステアリング角度からロボットの移動を計算します。
- ロボットの位置と姿勢を更新します。
- swerveおよびomniシャーシタイプをサポートします。

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| chassis_type | std::string | "" | シャーシタイプ（"swerve"または"omni"） |
| wheel_names | std::vector<std::string> | {""} | ホイールジョイント名のリスト |
| wheel_radius | double | 0.05 | ホイールの半径（m） |
| frequency | double | 1000.0 | シミュレーションの周期（Hz） |
| initial_pose_x | double | 0.0 | 初期位置のx座標（m） |
| initial_pose_y | double | 0.0 | 初期位置のy座標（m） |
| initial_pose_yaw_deg | double | 0.0 | 初期位置のヨー角（度） |
| wheel_base_names | std::vector<std::string> | {""} | ホイールベースジョイント名のリスト |
| infinite_swerve_mode | bool | false | 無限ステアリングモード（swerveのみ） |
| joint_names | std::vector<std::string> | {""} | ジョイント名のリスト |
| control_modes | std::vector<std::string> | {""} | 各ジョイントの制御モード（"position"または"speed"） |
| initial_positions | std::vector<double> | {0.0} | 各ジョイントの初期位置 |
| joint_position_tau | std::vector<double> | {0.5} | 位置制御の時定数 |
| joint_velocity_tau | std::vector<double> | {0.1} | 速度制御の時定数 |
| joint_velocity_max | std::vector<double> | {10.0} | 各ジョイントの最大速度 |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| /joint_states | sensor_msgs::msg::JointState | シミュレートされたジョイント状態 |
| /simulation_pose | geometry_msgs::msg::PoseStamped | ロボットの現在の位置と姿勢 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| /command_joint_states | sensor_msgs::msg::JointState | コマンドジョイント状態 |

# lidar_simulator
2D LiDARセンサーの動作を模擬します。

## 機能
- ロボットの位置と姿勢に基づいてレーザースキャンデータを生成します。
- 環境マップに基づいてレーザービームの衝突を計算します。

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| position_x | double | 0.0 | LiDARのx座標（m） |
| position_y | double | 0.0 | LiDARのy座標（m） |
| position_z | double | 0.0 | LiDARのz座標（m） |
| angle_deg | double | 0.0 | LiDARの取り付け角度（度） |
| range_min | double | 0.0 | 最小測定距離（m） |
| range_max | double | 35.0 | 最大測定距離（m） |
| angle_min | double | -1.57 | 最小測定角度（rad） |
| angle_max | double | 1.57 | 最大測定角度（rad） |
| simulation_resolution | double | 0.01 | シミュレーションの解像度（m） |
| point_rate | int | 43200 | 1秒あたりのポイント数 |
| frequency | double | 30 | スキャン周波数（Hz） |
| frame_id | std::string | "laser_frame" | LiDARフレームのID |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| /laserscan | sensor_msgs::msg::LaserScan | シミュレートされたレーザースキャンデータ |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| /simulation_pose | geometry_msgs::msg::PoseStamped | ロボットの現在の位置と姿勢 |
| /map | natto_msgs::msg::Map | 環境マップデータ |


# two_wheel_simulator
二輪駆動ロボットの動作を模擬します。
## 機能
- ホイールの速度からロボットの移動を計算します（差動駆動）。
- ロボットの位置と姿勢を更新します。

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| wheel_radius | double | 0.05 | ホイールの半径（m） |
| wheel_base | double | 0.5 | 左右のホイール間の距離（m） |
| wheel_speed_gain_p | double | 300.0 | ホイール速度の比例ゲイン |
| wheel_speed_gain_d | double | 100.0 | ホイール速度の微分ゲイン |
| frequency | double | 1000.0 | シミュレーションの周期（Hz） |
| initial_pose_x | double | 1.0 | 初期位置のx座標（m） |
| initial_pose_y | double | 1.0 | 初期位置のy座標（m） |
| initial_pose_yaw_deg | double | 0.0 | 初期位置のヨー角（度） |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| two_wheel_result | natto_msgs::msg::TwoWheel | シミュレートされたホイールの速度 |
| simulation_pose | geometry_msgs::msg::PoseStamped | ロボットの現在の位置と姿勢 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| two_wheel_command | natto_msgs::msg::TwoWheel | ホイールの目標速度 |
