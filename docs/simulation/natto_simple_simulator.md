# natto_simple_simulator
簡易シミュレータはロボットの基本的な動作を模擬します

# omni_simulator
オムニホイールロボットの動作を模擬します。
## 機能
- ホイールの速度からロボットの移動を計算します。
- ロボットの位置と姿勢を更新します。

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| wheel_radius | double | 0.05 | ホイールの半径（m） |
| wheel_position_x | std::vector<double> | {0.5, -0.5, -0.5, 0.5} | 各ホイールのx座標（m） |
| wheel_position_y | std::vector<double> | {0.5, 0.5, -0.5, -0.5} | | 各ホイールのy座標（m） |
| wheel_angle | std::vector<double> | {-45.0, 45.0, 135.0, -135.0} | 各ホイールの取り付け角度（度） |
| speed_gain_p | double | 100.0 | ホイール速度の比例ゲイン |
| speed_gain_d | double | 0.0 | ホイール速度の微分ゲイン |
| frequency | double | 1000.0 | シミュレーションの周期（Hz） |
| initial_pose_x | double | 1.0 | 初期位置のx座標（m） |
| initial_pose_y | double | 1.0 | 初期位置のy座標（m） |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| /omni_result | natto_msgs::msg::Omni | シミュレートされたホイールの速度 |
| /simulation_pose | geometry_msgs::msg::PoseStamped | ロボットの現在の位置と姿勢 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| /omni_command | natto_msgs::msg::Omni | ホイールの目標速度 |

# mecanum_simulator
メカナムホイールロボットの動作を模擬します。
## 機能
- ホイールの速度からロボットの移動を計算します。
- ロボットの位置と姿勢を更新します。

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| wheel_radius | double | 0.05 | ホイールの半径（m） |
| wheel_position_x | std::vector<double> | {0.5, -0.5, -0.5, 0.5} | 各ホイールのx座標（m） |
| wheel_position_y | std::vector<double> | {0.5, 0.5, -0.5, -0.5} | 各ホイールのy座標（m） |
| wheel_angle | std::vector<double> | {-45.0, 45.0, 135.0, -135.0} | 各ホイールの取り付け角度（度） |
| wheel_speed_gain_p | double | 300.0 | ホイール速度の比例ゲイン |
| wheel_speed_gain_d | double | 100.0 | ホイール速度の微分ゲイン |
| frequency | double | 1000.0 | シミュレーションの周期（Hz） |
| initial_pose_x | double | 1.0 | 初期位置のx座標（m） |
| initial_pose_y | double | 1.0 | 初期位置のy座標（m） |
| initial_pose_yaw_deg | double | 0.0 | 初期位置のヨー角（度） |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| /mecanum_result | natto_msgs::msg::Mecanum | シミュレートされたホイールの速度 |
| /simulation_pose | geometry_msgs::msg::PoseStamped | ロボットの現在の位置と姿勢 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| /mecanum_command | natto_msgs::msg::Mecanum | ホイールの目標速度 |

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
| frequency | double | 1000.0 | シミュレーションの周期（Hz） |
| initial_pose_x | double | 1.0 | 初期位置のx座標（m） |
| initial_pose_y | double | 1.0 | 初期位置のy座標（m） |
| initial_pose_yaw_deg | double | 0.0 | 初期位置のヨー角（度） |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| /swerve_result | natto_msgs::msg::Swerve | シミュレートされたホイールの角度と速度 |
| /simulation_pose | geometry_msgs::msg::PoseStamped | ロボットの現在の位置と姿勢 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| /swerve_command | natto_msgs::msg::Swerve | ホイールの目標角度と速度 |


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
