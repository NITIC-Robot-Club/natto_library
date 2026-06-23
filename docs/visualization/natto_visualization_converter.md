# natto_visualization_converter
natto_visualization_converter パッケージは、natto_msgs を可視化用のメッセージに変換してパブリッシュするノードを提供します。

# footprint_publisher
footprint_publisher ノードは、ロボットのフットプリントパラメーターからPolygon メッセージを生成し、パブリッシュします。

## 機能
- フットプリントの頂点座標をパラメーターから取得
- geometry_msgs/msg/PolygonStamped メッセージとしてパブリッシュ

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| footprint_points_x | double[] | [0.5, 0.5, -0.5, -0.5] | フットプリントの頂点のX座標 |
| footprint_points_y | double[] | [0.5, -0.5, -0.5, 0.5] | フットプリントの頂点のY座標 |
| frame_id | string | "base_link" | フットプリントのフレームID |
| frequency | double | 100.0 | フットプリントをパブリッシュする周期（Hz） |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| footprint | geometry_msgs/msg/PolygonStamped | ロボットのフットプリント |



# line_segment_visualizer
line_segment_visualizer ノードは、natto_msgs/msg/LineSegmentArray メッセージを受信し、可視化用の Marker メッセージに変換してパブリッシュします。

## 機能
- natto_msgs/msg/LineSegmentArray メッセージを受信
- 直線セグメント情報を可視化用の Marker メッセージに変換
- visualization_msgs/msg/MarkerArray メッセージとしてパブリッシュ

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| frequency | double | 100.0 | マーカー配列をパブリッシュする周期（Hz） |
| line_width | double | 0.05 | 直線セグメントの線幅 |
| frame_id | string | "" | マーカーのフレームID |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| marker_array | visualization_msgs/msg/MarkerArray | 変換した直線セグメント情報のマーカー配列 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| line_segments | natto_msgs/msg/LineSegmentArray | 直線セグメント情報 |



# line_visualizer
line_visualizer ノードは、natto_msgs/msg/LineArray メッセージを受信し、可視化用の Marker メッセージに変換してパブリッシュします。

## 機能
- natto_msgs/msg/LineArray メッセージを受信
- 直線情報を可視化用の Marker メッセージに変換
- visualization_msgs/msg/MarkerArray メッセージとしてパブリッシュ

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| frequency | double | 100.0 | マーカー配列をパブリッシュする周期（Hz） |
| line_length | double | 10.0 | 直線の長さ |
| line_width | double | 0.05 | 直線の線幅 |
| frame_id | string | "" | マーカーのフレームID |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| marker_array | visualization_msgs/msg/MarkerArray | 変換した直線情報のマーカー配列 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| lines | natto_msgs/msg/LineArray | 直線情報 |



# map_visualizer
map_visualizer ノードは、natto_msgs/msg/Map メッセージを受信し、可視化用の MarkerArray メッセージに変換してパブリッシュします。

## 機能
- natto_msgs/msg/Map メッセージを受信
- 直線セグメントと円弧情報を可視化用の Marker メッセージに変換
- visualization_msgs/msg/MarkerArray メッセージとしてパブリッシュ

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| frequency | double | 100.0 | マーカー配列をパブリッシュする周期（Hz） |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| marker_array | visualization_msgs/msg/MarkerArray | 変換した地図情報のマーカー配列 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| map | natto_msgs/msg/Map | 地図情報 |


# speed_path_visualizer
speed_path_visualizer ノードは、natto_msgs/msg/SpeedPath メッセージを受信し、可視化用の MarkerArray メッセージに変換してパブリッシュします。

## 機能
- natto_msgs/msg/SpeedPath メッセージを受信
- 経路・速度・方向を可視化用の Marker メッセージに変換
- visualization_msgs/msg/MarkerArray メッセージとしてパブリッシュ

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| frequency | double | 100.0 | マーカー配列をパブリッシュする周期（Hz） |
| path_width | double | 0.05 | 経路ラインの幅 |
| frame_id | string | "map" | マーカーのフレームID |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| marker_array | visualization_msgs/msg/MarkerArray | 変換したSpeedPath情報のマーカー配列 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| speed_path | natto_msgs/msg/SpeedPath | スピードパスデータ |


# swerve_visualizer
swerve_visualizer ノードは、`command_joint_states` を受信し、swerve シャーシの各ホイールの向きと速度をベクトルとして可視化します。

## 機能
- `chassis_calculator` と同じパラメータ名で swerve 用の設定を受け取る
- 各ホイールのステア角と車輪速度から、長さ付きの矢印を生成
- 連続する `command_joint_states` のステア角差分を少しだけ平滑化し、車体前方 0° を始点にした円弧状ステア速マーカーを生成
- `visualization_msgs/msg/MarkerArray` としてパブリッシュ
- RViz では `/visualization/steering_vector` の `MarkerArray` ディスプレイで確認できる

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| chassis_type | string | "" | シャーシタイプ（`swerve` のみ対応） |
| wheel_radius | double | 0.05 | ホイール半径（m）。回転速度を表示長に変換するために使用 |
| wheel_names | string[] | {""} | ホイールジョイント名のリスト |
| wheel_base_names | string[] | {""} | ホイールベースジョイント名のリスト |
| infinite_swerve_mode | bool | false | `chassis_calculator` 互換のための設定 |
| frequency | double | 100.0 | マーカー配列をパブリッシュする周期（Hz） |
| frame_id | string | "command/base_link" | マーカーのフレームID |
| line_width | double | 0.05 | 矢印の太さ |
| vector_scale | double | 0.25 | 車輪速度に掛ける表示スケール |
| rotation_vector_scale | double | 0.12 | 円弧の半径スケール |
| rotation_vector_line_width | double | 0.03 | 円弧と矢印の線幅 |
| steering_speed_history_length | int | 5 | ステア速の履歴平均に使うサンプル数 |
| steering_speed_render_alpha | double | 0.16 | ステア速の描画追従量。大きいほど速く追従する |

## 調整の目安
- `steering_speed_history_length` を増やすと円弧の速度表示がなめらかになり、減らすと生の変化に近づく
- `steering_speed_render_alpha` を上げると表示が速く追従し、下げると残像が長くなる
- 低速域を見やすくしたい場合は `steering_speed_render_alpha` を少し下げる
- 平滑化の影響で表示は実際の変化から少し遅れて見えるため、値のズレが気になる場合は `steering_speed_history_length` と `steering_speed_render_alpha` を調整する


## launch での調整
- `visualization.launch.xml` に `steering_speed_history_length` と `steering_speed_render_alpha` を引数として追加してあるので、起動時に上書きできる
- 例: `ros2 launch natto_launch visualization.launch.xml steering_speed_history_length:=3 steering_speed_render_alpha:=0.25`


## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| marker_array | visualization_msgs/msg/MarkerArray | 変換したステアベクトル |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| command_joint_states | sensor_msgs/msg/JointState | `chassis_calculator` の出力 |
