# natto_wheel_odometry
このパッケージは計測輪や足回りの出力結果からオドメトリを計算します

# swerve_odometry
swerve_odometryノードは独ステのオドメトリを計算します

## 機能
- 独立ステアリング機構を持つロボットのオドメトリ計算
- 各ホイールの速度とステアリング角度から位置と姿勢を推定

## パラメータ
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| wheel_radius | double | 0.05 | ホイールの半径（m） |
| wheel_position_x | double[] | [0.5, -0.5, -0.5, 0.5] | 各ホイールのx座標位置（m） |
| wheel_position_y | double[] | [0.5, 0.5, -0.5, -0.5] | 各ホイールのy座標位置（m） |
| frame_id | string | "odom" | オドメトリのフレームID | |
| child_frame_id | string | "base_link" | オドメトリの子フレームID | |
| publish_tf | bool | true | TFをパブリッシュするかどうか |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| twist | geometry_msgs::msg::TwistStamped | 計算されたツイスト |
| pose | geometry_msgs::msg::PoseStamped | 計算されたポーズ |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| swerve | natto_msgs::msg::Swerve | 独ステ結果 |

# two_wheel_odometry
two_wheel_odometryノードは二輪駆動のオドメトリを計算します

## 機能
- 二輪駆動機構を持つロボットのオドメトリ計算
- 各ホイールの速度から位置と姿勢を推定（差動駆動キネマティクス）

## パラメータ
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| wheel_radius | double | 0.05 | ホイールの半径（m） |
| wheel_base | double | 0.5 | 左右のホイール間の距離（m） |
| odom_frame_id | string | "odom" | オドメトリのフレームID |
| base_frame_id | string | "base_link" | オドメトリの子フレームID |
| publish_tf | bool | true | TFをパブリッシュするかどうか |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| twist | geometry_msgs::msg::TwistStamped | 計算されたツイスト |
| pose | geometry_msgs::msg::PoseStamped | 計算されたポーズ |
| odometry | nav_msgs::msg::Odometry | オドメトリ情報 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| two_wheel_result | natto_msgs::msg::TwoWheel | 二輪駆動結果 |
