# natto_command_selector
twist_selectorノードは、手動運転と自動運転の移動コマンドを受け取り、現在の運転モードに応じて適切なコマンドを選択してパブリッシュします。

# twist_selector
twist_selectorノードは、手動運転と自動運転の移動コマンドを受信し、運転モードに応じてコマンドを選択してパブリッシュします。

## 機能
- 手動運転と自動運転の移動コマンドを受信
- 運転モードに応じてコマンドを選択し、パブリッシュ

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| initial_allow_auto_drive | bool | false | ノード起動時の自動運転許可状態 |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| selected_twist | geometry_msgs/msg/TwistStamped | 選択された移動コマンド |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| manual_twist | geometry_msgs/msg/TwistStamped | 手動運転の移動コマンド |
| auto_twist | geometry_msgs/msg/TwistStamped | 自動運転の移動コマンド |
| allow_auto_drive | std_msgs/msg/Bool | 自動運転の許可状態 |
