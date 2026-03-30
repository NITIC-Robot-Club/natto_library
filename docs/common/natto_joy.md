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
| button_N.functions | string[] | [] | ボタンNの機能リスト（`function` より優先）。1ボタンで複数機能を実行可能 |
| button_N.joint_name | string | "" | 旧仕様: ボタンNが制御するジョイント名（単一エントリ時） |
| button_N.joint_names | string[] | [] | 複数エントリ時のジョイント名リスト（`functions` と同じ添字で対応） |
| button_N.position_on | double | 1.0 | 単一エントリ時のON目標位置 |
| button_N.position_off | double | 0.0 | 単一エントリ時のOFF目標位置 |
| button_N.speed_on | double | 1.0 | 単一エントリ時のON目標速度 |
| button_N.speed_off | double | 0.0 | 単一エントリ時のOFF目標速度 |
| button_N.publish_always | bool | false | 単一エントリ時に自動運転中もパブリッシュするか |
| button_N.position_ons | double[] | [] | 複数エントリ時のON目標位置リスト |
| button_N.position_offs | double[] | [] | 複数エントリ時のOFF目標位置リスト |
| button_N.speed_ons | double[] | [] | 複数エントリ時のON目標速度リスト |
| button_N.speed_offs | double[] | [] | 複数エントリ時のOFF目標速度リスト |
| button_N.publish_always_list | bool[] | [] | 複数エントリ時の自動運転中パブリッシュ設定リスト |
| button_N.priority | int | 0 | 単一エントリ時の優先度 |
| button_N.priorities | int[] | [] | 複数エントリ時の優先度リスト |
| zr.mode | string | "none" | ZRトリガーの動作モード |
| zr.function | string | "none" | ZRトリガーの機能 |
| zr.functions | string[] | [] | ZRの機能リスト（`function` より優先） |
| zr.joint_name | string | "" | 単一エントリ時のZR対象ジョイント名 |
| zr.joint_names | string[] | [] | 複数エントリ時のZR対象ジョイント名リスト |
| zr.position_on | double | 1.0 | 単一エントリ時のZR ON目標位置 |
| zr.position_off | double | 0.0 | 単一エントリ時のZR OFF目標位置 |
| zr.speed_on | double | 1.0 | 単一エントリ時のZR ON目標速度 |
| zr.speed_off | double | 0.0 | 単一エントリ時のZR OFF目標速度 |
| zr.publish_always | bool | false | 単一エントリ時のZR自動運転中パブリッシュ設定 |
| zr.position_ons | double[] | [] | 複数エントリ時のZR ON目標位置リスト |
| zr.position_offs | double[] | [] | 複数エントリ時のZR OFF目標位置リスト |
| zr.speed_ons | double[] | [] | 複数エントリ時のZR ON目標速度リスト |
| zr.speed_offs | double[] | [] | 複数エントリ時のZR OFF目標速度リスト |
| zr.publish_always_list | bool[] | [] | 複数エントリ時のZR自動運転中パブリッシュ設定 |
| zl.mode | string | "none" | ZLトリガーの動作モード |
| zl.function | string | "none" | ZLトリガーの機能 |
| zl.functions | string[] | [] | ZLの機能リスト（`function` より優先） |
| zl.joint_name | string | "" | 単一エントリ時のZL対象ジョイント名 |
| zl.joint_names | string[] | [] | 複数エントリ時のZL対象ジョイント名リスト |
| zl.position_on | double | 1.0 | 単一エントリ時のZL ON目標位置 |
| zl.position_off | double | 0.0 | 単一エントリ時のZL OFF目標位置 |
| zl.speed_on | double | 1.0 | 単一エントリ時のZL ON目標速度 |
| zl.speed_off | double | 0.0 | 単一エントリ時のZL OFF目標速度 |
| zl.publish_always | bool | false | 単一エントリ時のZL自動運転中パブリッシュ設定 |
| zl.position_ons | double[] | [] | 複数エントリ時のZL ON目標位置リスト |
| zl.position_offs | double[] | [] | 複数エントリ時のZL OFF目標位置リスト |
| zl.speed_ons | double[] | [] | 複数エントリ時のZL ON目標速度リスト |
| zl.speed_offs | double[] | [] | 複数エントリ時のZL OFF目標速度リスト |
| zl.publish_always_list | bool[] | [] | 複数エントリ時のZL自動運転中パブリッシュ設定 |

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

### `button_N.functions` の使用例

```yaml
button_0:
	mode: "hold"
	functions: ["joint_speed", "joint_position"]
	joint_names: ["joint_1", "joint_2"]
	speed_ons: [1.0, 0.0]
	speed_offs: [0.0, 0.0]
	position_ons: [0.0, 1.0]
	position_offs: [0.0, 0.0]
```

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