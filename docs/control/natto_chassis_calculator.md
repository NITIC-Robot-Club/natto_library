# natto_chassis_calculator
natto_chassis_calculator パッケージは、ロボットの走行制御に関する計算ノードを提供します。オムニホイールと独立ステアリングの両方の計算をサポートします。

# chassis_calculator
chassis_calculator ノードは、ロボットの指令速度から各ホイールの角度と速度を計算します。

## 機能
- geometry_msgs/msg/TwistStamped メッセージを受信
- chassis_typeに応じて各ホイールの角度と速度を計算
- sensor_msgs/msg/JointState メッセージとしてパブリッシュ
- swerveおよびomniシャーシタイプをサポート

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| chassis_type | std::string | "" | シャーシタイプ（"swerve"または"omni"または"メカナム"） |
| wheel_radius | double | 0.05 | ホイールの半径（m） |
| wheel_names | std::vector<std::string> | {""} | ホイールジョイント名のリスト |
| wheel_base_names | std::vector<std::string> | {""} | ホイールベースジョイント名のリスト |
| infinite_swerve_mode | bool | false | 無限スウェーブモード（swerveのみ） |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| command_joint_states | sensor_msgs/msg/JointState | 計算されたホイールの角度と速度 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| command_velocity | geometry_msgs/msg/TwistStamped | ロボットの指令速度 |

