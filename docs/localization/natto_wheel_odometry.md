# natto_wheel_odometry
このパッケージは計測輪や足回りの出力結果からオドメトリを計算します

# wheel_odometry
wheel_odometryノードは様々な足回り機構のオドメトリを計算します

## 機能
- sensor_msgs/msg/JointStateメッセージからオドメトリを計算
- chassis_typeに応じて適切な計算方法を選択
- URDFからホイールの位置と角度を動的に取得

## パラメータ
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| chassis_type | string | "" | シャーシのタイプ（"swerve", "omni", "mecanum"） |
| wheel_radius | double | 0.05 | ホイールの半径（m） |
| wheel_names | std::vector<string> | {""} | ホイールのジョイント名のリスト |
| wheel_base_names | std::vector<string> | {""} | ホイールベースのジョイント名のリスト |
| odom_frame_id | string | "odom" | オドメトリのフレームID |
| base_frame_id | string | "base_link" | オドメトリの子フレームID |
| publish_tf | bool | false | TFをパブリッシュするかどうか |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| twist | geometry_msgs::msg::TwistStamped | 計算されたツイスト |
| pose | geometry_msgs::msg::PoseStamped | 計算されたポーズ |
| odometry | nav_msgs::msg::Odometry | 計算されたオドメトリ |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| joint_states | sensor_msgs::msg::JointState | ホイールのジョイント状態 |

## 対応シャーシタイプ

### swerve (独立ステアリング)
独立ステアリング機構を持つロボットのオドメトリ計算。各ホイールの速度とステアリング角度から位置と姿勢を推定します。

### omni (オムニホイール)  
オムニホイール機構を持つロボットのオドメトリ計算。最小二乗法を用いて各ホイールの速度から位置と姿勢を推定します。

### mecanum (メカナムホイール)
メカナムホイール機構を持つロボットのオドメトリ計算。オムニホイールと同じ最小二乗法アプローチを使用します。

## 計算方法

オムニホイールとメカナムホイールでは、以下の最小二乗法を使用して速度を推定します：

各ホイールについて、以下の式が成り立ちます：
```
wheel_speed = vx * cos(wheel_angle) + vy * sin(wheel_angle) + vyaw * (-wheel_pos_y * cos(wheel_angle) + wheel_pos_x * sin(wheel_angle))
```

これをA^T·A·x = A^T·bの形で解くことで、vx, vy, vyawを求めます。
