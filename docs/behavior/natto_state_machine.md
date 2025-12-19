# natto_state_machine
状態遷移を管理

# default_action
デフォルトのアクションノード。

## 機能
- `set_pose`アクションを受け取り、目標位置と姿勢を設定する
- `goal_reached`トピックから現在の到達状況を受け取り、目標に到達したかどうかを判定する
- 位置と姿勢の許容範囲をパラメータで設定可能

## パラメータ
| パラメータ名         | 型     | デフォルト値 | 説明                     |
| -                   | -     | -          | -                           |
| xy_tolerance_m      | double | 0.2        | 位置の許容範囲（メートル）       |
| yaw_tolerance_deg   | double | double | 10.0       | 姿勢の許容範囲（度）          |

## サブスクライバー
| トピック名     | メッセージ型                      | 説明                     |
| -             | -                                | -                       |
| state_action  | natto_msgs/msg/StateAction       | 状態アクションの受信           |
| goal_reached  | std_msgs/msg/Bool                | 目標到達状況の受信           |
| current_pose  | geometry_msgs/msg/PoseStamped     | 現在の位置と姿勢の受信         |  

## パブリッシャー
| トピック名      | メッセージ型                      | 説明                     |
| -              | -                                | -                       |
| state_result   | natto_msgs/msg/StateResult       | 状態アクションの結果送信         |    
| goal_pose     | geometry_msgs/msg/PoseStamped     | 目標位置と姿勢の送信           |

# mermaid_loader
Mermaid形式の状態遷移図に基づいて状態遷移を管理

## 機能
- Mermaid形式の状態遷移図を解析し、状態遷移を管理

## パラメータ
| パラメータ名         | 型     | デフォルト値 | 説明                     |
| -                   | -     | -          | -                           |
| mermaid_path    | string | ""         | Mermaid形式の状態遷移図　|

## パブリッシャー
| トピック名      | メッセージ型                      | 説明                     |
| -              | -                                | -                       |
| state_graph    | natto_msgs/msg/StateGraph        | 状態遷移図の送信           |

# state_machine
状態遷移を管理するノード

## 機能
- 各状態のアクションを実行し、結果に基づいて次の状態に遷移
- 状態遷移の条件を設定可能  

## パラメータ
| パラメータ名         | 型     | デフォルト値 | 説明                     |
| -                   | -     | -          | -                           |
| state_timeout_sec| double    | 1.0          | アクションのタイムアウト時間       |

## サブスクライバー
| トピック名     | メッセージ型                      | 説明                     |
| -             | -                                | -                       |
| state_result  | natto_msgs/msg/StateResult       | 状態アクションの結果受信           |
| force_state | std_msgs/msg/UInt64              | 強制的に状態を設定するための受信       |
| state_graph    | natto_msgs/msg/StateGraph        | 状態遷移図の受信           |    

## パブリッシャー
| トピック名      | メッセージ型                      | 説明                     |
| -              | -                                | -                       |
| state_action   | natto_msgs/msg/StateAction       | 状態アクションの送信         |
