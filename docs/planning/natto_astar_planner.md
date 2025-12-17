# natto_astar_planner
A*アルゴリズムに基づく経路計画ノードです。
x,y,yawの計画を行います

## 機能
- 2Dグリッドマップ上でのA*経路計画
- 角度を考慮した経路計画
- 経路のスムージング
- 凹凸のフットプリントを考慮した計画

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