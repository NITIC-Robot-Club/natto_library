# natto_chassis_calculator
natto_chassis_calculator パッケージは、ロボットの走行制御に関する計算ノードを提供します。現在、オムニホイール、独立ステアリング、二輪駆動の計算ノードが含まれています。

# swerve_calculator
swerve_calculator ノードは、ロボットの指令速度から各ホイールの角度と速度を計算します

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


# omni_calculator
omni_calculator ノードは、ロボットの指令速度からオムニホイールの各ホイール速度を計算します

## 機能
- geometry_msgs/msg/TwistStamped メッセージを受信
- 各ホイールの速度を計算
- natto_msgs/msg/Omni メッセージとしてパブリッシュ

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| wheel_radius | double | 0.05 | ホイールの半径 |
| wheel_position_x | std::vector<double> | {0.5, -0.5, -0.5, 0.5} | ホイールのX座標 |
| wheel_position_y | std::vector<double> | {0.5, 0.5, -0.5, -0.5} | ホイールのY座標 |
| wheel_angle | std::vector<double> | {-45.0, 45.0, 135.0, -135.0} | ホイールの取り付け角度（度） |

※ホイールの取り付け角度は、ロボット前方を0度として反時計回りに測定します。
例：　左前に斜め45度で取り付けられている場合、wheel_position_x,yはどちらも正の値となり、wheel_angleは-45.0度となります。
wheel_angleを元にタイヤの回転方向を計算します。
もし逆方向に回転する場合180度を加算してください。

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| omni_command | natto_msgs/msg/Omni | 計算された各ホイール速度 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| command_velocity | geometry_msgs/msg/TwistStamped | ロボットの指令速度 |


# two_wheel_calculator
two_wheel_calculator ノードは、ロボットの指令速度から二輪駆動の各ホイール速度を計算します

## 機能
- geometry_msgs/msg/TwistStamped メッセージを受信
- 各ホイールの速度を計算（横移動は無効化）
- natto_msgs/msg/TwoWheel メッセージとしてパブリッシュ

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| wheel_radius | double | 0.05 | ホイールの半径 |
| wheel_base | double | 0.5 | 左右のホイール間の距離 |

※二輪駆動は差動駆動キネマティクスを使用します。
横移動（Y軸）は無効化され、前後移動（X軸）と回転（Z軸）のみが考慮されます。

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| two_wheel_command | natto_msgs/msg/TwoWheel | 計算された各ホイール速度 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| command_velocity | geometry_msgs/msg/TwistStamped | ロボットの指令速度 |
