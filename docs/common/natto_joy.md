# natto_joy
joyトピックを元にロボットを制御します

# button_manager
button_managerノードは、ジョイスティックのボタン入力を管理し、特定のボタン操作に応じてロボットの動作モードを切り替えます。

## 機能
- 電源のオン/オフ　power
- 自動運転許可/禁止　allow_auto_drive

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| power_on_button | int | 0 | 電源オンにするボタンのインデックス |
| power_off_button | int | 1 | 電源オフにするボタンのインデックス |
| allow_auto_drive_method | string | "toggle" | 自動運転許可/禁止の切り替え方法 ("toggle" または "hold") |
| allow_auto_drive_on_button | int | 2 | 自動運転許可にするボタンのインデックス |
| allow_auto_drive_off_button | int | 3 | | 自動運転禁止にするボタンのインデックス |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| power | std_msgs/msg/Bool | ロボットの電源状態 |
| allow_auto_drive | std_msgs/msg/Bool | 自動運転の許可状態 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| joy | sensor_msgs/msg/Joy | ジョイスティックの入力データ |

# joy_to_cmd_vel
joy_to_cmd_velノードは、ジョイスティックの入力を受け取り、ロボットの移動コマンド（geometry_msgs/msg/Twist型）に変換してパブリッシュします。

## 機能
- sensor_msgs/msg/Joy型のジョイスティック入力を受信
- geometry_msgs/msg/TwistStamped型の移動コマンドをパブリッシュ

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| max_xy_speed_m_s | double | 2.0 | 前後・左右移動の最大速度 (m/s) |
| max_theta_speed_rad_s | double | 3.1415 | 回転の最大速度 (rad/s) |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| command_velocity | geometry_msgs/msg/TwistStamped | ロボットの移動コマンド |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| joy | sensor_msgs/msg/Joy | ジョイスティックの入力データ |