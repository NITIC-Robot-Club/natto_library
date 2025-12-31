# natto_wheel_odometry
このパッケージは計測輪や足回りの出力結果からオドメトリを計算します

# wheel_odometry
wheel_odometryノードはジョイント状態からオドメトリを計算します

## 機能
- 独立ステアリングおよびオムニホイール機構を持つロボットのオドメトリ計算
- 各ホイールの速度とステアリング角度から位置と姿勢を推定
- chassis_typeパラメーターでswerveとomniをサポート

## パラメータ
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| chassis_type | std::string | "" | シャーシタイプ（"swerve"または"omni"またはメカナム） |
| wheel_radius | double | 0.05 | ホイールの半径（m） |
| wheel_names | std::vector<std::string> | {""} | ホイールジョイント名のリスト |
| wheel_base_names | std::vector<std::string> | {""} | ホイールベースジョイント名のリスト |
| odom_frame_id | std::string | "odom" | オドメトリのフレームID |
| base_frame_id | std::string | "base_link" | オドメトリの子フレームID |
| publish_tf | bool | false | TFをパブリッシュするかどうか |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| twist | geometry_msgs::msg::TwistStamped | 計算されたツイスト |
| pose | geometry_msgs::msg::PoseStamped | 計算されたポーズ |
| odometry | nav_msgs::msg::Odometry | オドメトリ情報 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| joint_states | sensor_msgs::msg::JointState | ジョイント状態 |
