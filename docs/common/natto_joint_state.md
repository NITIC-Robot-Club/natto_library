# natto_joint_state
複数のジョイント状態の結合や制御ルールの適用を行うノードを提供します。

# joint_state_merger
joint_state_merger ノードは、複数のトピックから受信した JointState メッセージをマージして単一の JointState としてパブリッシュします。

## 機能
- 複数の JointState トピックをまとめて1つの JointState に統合
- 同名ジョイントは上書き、新規ジョイントは追記

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| joint_state_topics | string[] | [""] | 統合対象のJointStateトピック名リスト |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| merged_joint_states | sensor_msgs/msg/JointState | 統合されたジョイント状態 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| `joint_state_topics` で指定したトピック | sensor_msgs/msg/JointState | 各ジョイント状態 |

# joint_state_rule
joint_state_rule ノードは、ジョイントの目標状態に対してルールに基づく位置クランプ処理を行います。

## 機能
- 現在のジョイント状態または指令状態が指定範囲に入っているとき、別ジョイントの指令値をクランプ
- YAML パラメーターでルールを定義可能

## パラメーター
パラメーターは動的に宣言されます（`allow_undeclared_parameters`）。

| パラメーター名 | 型 | 説明 |
| - | - | - |
| rules.RULE.if_COND.joint_name | string | 条件となるジョイント名 |
| rules.RULE.if_COND.range | double[2] | 条件範囲 [min, max]（ラジアン） |
| rules.RULE.then.joint_name | string | クランプ対象のジョイント名 |
| rules.RULE.then.range | double[2] | クランプ範囲 [min, max]（ラジアン） |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| fixed_command_joint_states | sensor_msgs/msg/JointState | ルール適用後のジョイント指令状態 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| joint_states | sensor_msgs/msg/JointState | 現在のジョイント状態 |
| command_joint_states | sensor_msgs/msg/JointState | ジョイント指令状態 |
