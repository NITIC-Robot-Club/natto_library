# natto_speed_path
natto_speed_path パッケージは、CSVファイルから SpeedPath を読み込み、状態機械のアクションとして提供するノードを提供します。

# speed_path_loader
speed_path_loader ノードは、`load_speed_path` アクションを受信してCSVファイルを読み込み、natto_msgs/msg/SpeedPath をパブリッシュします。

## 機能
- `state_action` から `load_speed_path` アクションを受信し、CSVファイルを読み込む
- `goal_reached` を受信すると SpeedPath をクリアし、`state_result` で成功を通知
- 定期的に現在の SpeedPath をパブリッシュ

## CSVファイルフォーマット
1行目はヘッダ（スキップ）。2行目以降のカンマ区切りデータ。

| t | x | y | yaw | vx | vy | vz |
| - | - | - | - | - | - | - |
| 時刻（使用されない） | X座標（m） | Y座標（m） | ヨー角（rad） | X速度（m/s） | Y速度（m/s） | 角速度（rad/s） |

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| file_directory | string | "" | CSVファイルが格納されたディレクトリパス（必須） |
| frequency | double | 1.0 | SpeedPathをパブリッシュする周期（Hz） |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| speed_path | natto_msgs/msg/SpeedPath | 読み込まれたSpeedPath |
| state_result | natto_msgs/msg/StateResult | アクション実行結果 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| state_action | natto_msgs/msg/StateAction | 状態アクション（`load_speed_path` アクションを処理） |
| goal_reached | std_msgs/msg/Bool | ゴール到達通知（trueを受信するとSpeedPathをクリア） |
