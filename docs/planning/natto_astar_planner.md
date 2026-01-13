# natto_astar_planner
A*アルゴリズムに基づく経路計画ノードです。
x,y,yawの計画を行います

## 機能
- 2Dグリッドマップ上でのA*経路計画
- 角度を考慮した経路計画
- 経路のスムージング
- 凹凸のフットプリントを考慮した計画
- 経路からの逸脱を検知した自動再計画
- ゴール到達時の経路クリア

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| theta_resolution_deg | int | 15 | 角度の分解能（度） |
| xy_inflation | double | 0.5 | XY方向のインフレーション距離（m）この範囲は絶対に入れない |
| xy_offset | double | 0.1 | XY方向のオフセット距離（m）xy_inflationに追加される範囲、できるだけ入りたくない |
| yaw_offset | double | 0.1 | ヨー方向のオフセット距離（rad） |
| grad_alpha | double | 1.0 | 勾配法のアルファパラメーター。増やすと障害物から離れていく |
| grad_beta | double | 8.0 | 勾配法のベータパラメーター。増やすとなめらかな経路になる |
| grad_gamma | double | 0.0 | 勾配法のガンマパラメーター。増やすともとの経路に従いやすくなる |
| grad_step_size | double | 0.1 | 勾配法のステップサイズ |
| replan_distance_threshold | double | 0.2 | 経路再計画の距離しきい値（m）。ロボットが経路からこの距離以上離れると自動的に再計画を実行 |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| path | nav_msgs::msg::Path | 計画された経路 |
| costmap | nav_msgs::msg::OccupancyGrid | 使用されたコストマップ |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| occupancy_grid | nav_msgs::msg::OccupancyGrid | グリッドマップ |
| goal_pose | geometry_msgs::msg::PoseStamped | 目標位置 |
| current_pose | geometry_msgs::msg::PoseStamped | 現在位置 |
| footprint | geometry_msgs::msg::PolygonStamped | ロボットのフットプリント |
| goal_reached | std_msgs::msg::Bool | ゴール到達フラグ（trueを受信すると経路をクリア） |

## 動作詳細

### 経路再計画機能
10Hzのタイマーで定期的にロボットの現在位置と計画済み経路の最短距離を計算します。この距離が`replan_distance_threshold`パラメーターで設定された値を超えると、自動的に経路を再計画します。

また、同じゴールを再度受信した場合でも、ロボットが経路を追従できている（しきい値以内の距離）場合は、再計画を行わずに現在の経路を維持します。これにより、不要な再計画を防ぎ、スムーズな動作を実現します。

### ゴール到達時の処理
`goal_reached`トピックに`true`が送信されると、計画済みの経路をクリアします。これにより、ゴール到達後の不要な経路表示を防ぎます。