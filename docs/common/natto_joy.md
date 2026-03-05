# natto_joy
joyトピックを元にロボットを制御します

# button_manager
button_managerノードは、ジョイスティックのボタン入力を管理し、特定のボタン操作に応じてロボットの動作モードを切り替えます。

## 機能
- 電源のオン/オフ: `power`
- 自動運転許可/禁止: `allow_auto_drive`
- ジョイントの位置・速度制御: `joint_position`, `joint_speed`
- ジョイントのオリジン取得: `get_origin`

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| num_button | int | 0 | 管理するボタン数 |
| button_N.mode | string | "none" | ボタンNの動作モード（"toggle", "toggle_on", "toggle_off", "hold", "positive_edge", "none"） |
| button_N.function | string | "none" | ボタンNの機能（"power", "allow_auto_drive", "joint_position", "joint_speed", "get_origin", "none"） |
| button_N.joint_name | string | "" | ボタンNが制御するジョイント名（joint_position/joint_speed時に必須） |
| button_N.joint_names | string[] | [""] | ボタンNが対象とするジョイント名リスト（get_origin時に必須） |
| button_N.position_on | double | 1.0 | ボタンNがONのときのジョイント目標位置 |
| button_N.position_off | double | 0.0 | ボタンNがOFFのときのジョイント目標位置 |
| button_N.speed_on | double | 1.0 | ボタンNがONのときのジョイント目標速度 |
| button_N.speed_off | double | 0.0 | ボタンNがOFFのときのジョイント目標速度 |
| button_N.publish_always | bool | false | 自動運転中でもジョイント状態をパブリッシュするか |
| zr.mode | string | "none" | ZRトリガーの動作モード |
| zr.function | string | "none" | ZRトリガーの機能 |
| zr.joint_name | string | "" | ZRトリガーが制御するジョイント名 |
| zr.position_on | double | 1.0 | ZRがONのときのジョイント目標位置 |
| zr.position_off | double | 0.0 | ZRがOFFのときのジョイント目標位置 |
| zr.speed_on | double | 1.0 | ZRがONのときのジョイント目標速度 |
| zr.speed_off | double | 0.0 | ZRがOFFのときのジョイント目標速度 |
| zr.publish_always | bool | false | 自動運転中でもパブリッシュするか |
| zl.mode | string | "none" | ZLトリガーの動作モード |
| zl.function | string | "none" | ZLトリガーの機能 |
| zl.joint_name | string | "" | ZLトリガーが制御するジョイント名 |
| zl.position_on | double | 1.0 | ZLがONのときのジョイント目標位置 |
| zl.position_off | double | 0.0 | ZLがOFFのときのジョイント目標位置 |
| zl.speed_on | double | 1.0 | ZLがONのときのジョイント目標速度 |
| zl.speed_off | double | 0.0 | ZLがOFFのときのジョイント目標速度 |
| zl.publish_always | bool | false | 自動運転中でもパブリッシュするか |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| power | std_msgs/msg/Bool | ロボットの電源状態 |
| allow_auto_drive | std_msgs/msg/Bool | 自動運転の許可状態 |
| joint_states | sensor_msgs/msg/JointState | ボタン操作によるジョイント目標状態 |
| get_origin_joint_name | std_msgs/msg/String | オリジン取得するジョイント名 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| joy | sensor_msgs/msg/Joy | ジョイスティックの入力データ |

# joy_to_twist
joy_to_twistノードは、ジョイスティックの入力を受け取り、ロボットの移動コマンド（geometry_msgs/msg/TwistStamped型）に変換してパブリッシュします。

## 機能
- sensor_msgs/msg/Joy型のジョイスティック入力を受信
- geometry_msgs/msg/TwistStamped型の移動コマンドをパブリッシュ

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| max_xy_speed_m_s | double | 2.0 | 前後・左右移動の最大速度 (m/s) |
| max_yaw_speed_rad_s | double | 3.1415 | 回転の最大速度 (rad/s) |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| command_velocity | geometry_msgs/msg/TwistStamped | ロボットの移動コマンド |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| joy | sensor_msgs/msg/Joy | ジョイスティックの入力データ |