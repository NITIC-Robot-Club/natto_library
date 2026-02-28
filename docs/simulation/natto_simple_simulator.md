# natto_simple_simulator
簡易シミュレータはロボットの基本的な動作を模擬します

# joint_state_simulator
様々な足回り機構のシミュレーションを行います。

## 機能
- sensor_msgs/msg/JointStateメッセージを受信して目標値として使用
- ホイールの速度や角度を模擬
- ロボットの位置と姿勢を計算して更新
- chassis_typeに応じて適切なシミュレーションを実行

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| chassis_type | string | "" | シャーシのタイプ（"swerve", "omni", "mecanum"） |
| wheel_radius | double | 0.05 | ホイールの半径（m） |
| wheel_names | std::vector<string> | {""} | ホイールのジョイント名のリスト |
| wheel_base_names | std::vector<string> | {""} | ホイールベースのジョイント名のリスト |
| frequency | double | 1000.0 | シミュレーションの周期（Hz） |
| initial_pose_x | double | 0.0 | 初期位置のx座標（m） |
| initial_pose_y | double | 0.0 | 初期位置のy座標（m） |
| initial_pose_yaw_deg | double | 0.0 | 初期位置のヨー角（度） |
| joint_names | std::vector<string> | {""} | シミュレートするジョイントの名前 |
| control_modes | std::vector<string> | {""} | 各ジョイントの制御モード（"position" or "speed"） |
| initial_positions | std::vector<double> | {0.0} | 各ジョイントの初期位置 |
| joint_position_tau | std::vector<double> | {0.1} | 位置制御の時定数 |
| joint_velocity_tau | std::vector<double> | {0.1} | 速度制御の時定数 |
| joint_velocity_max | std::vector<double> | {60.0} | 各ジョイントの最大速度 |
| infinite_swerve_mode | bool | false | 独ステ用：無限回転モード |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| joint_states | sensor_msgs::msg::JointState | シミュレートされたジョイント状態 |
| simulation_pose | geometry_msgs::msg::PoseStamped | ロボットの現在の位置と姿勢 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| command_joint_states | sensor_msgs::msg::JointState | ジョイントの目標値 |

## 対応シャーシタイプ

### swerve (独立ステアリング)
独立ステアリング機構のロボットをシミュレート。

### omni (オムニホイール)
オムニホイール機構のロボットをシミュレート。

### mecanum (メカナムホイール)
メカナムホイール機構のロボットをシミュレート。オムニホイールと同じ計算式を使用します。

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
| angle | double | 0.0 | LiDARの取り付け角度（rad） |
| range_min | double | 0.0 | 最小測定距離（m） |
| range_max | double | 35.0 | 最大測定距離（m） |
| angle_min | double | -1.57 | 最小測定角度（rad） |
| angle_max | double | 1.57 | 最大測定角度（rad） |
| simulation_resolution | double | 0.01 | シミュレーションの解像度（m） |
| point_rate | int | 43200 | 1秒あたりのポイント数 |
| frequency | int | 30 | スキャン周波数（Hz） |
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
