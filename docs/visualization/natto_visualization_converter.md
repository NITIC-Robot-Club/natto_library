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
- `command_joint_states` からは、ホイール速度矢印とステア速度の円弧を `alpha 0.5` で描画する
- `/joint_states` からは、同じホイール速度矢印とステア速度の円弧を `alpha 1.0` で、command 側と同じ色で描画する
- 各ホイールのステア角と車輪速度から、長さ付きの矢印を生成する
- `show_steering_arc` を有効にすると、ステア速度に応じた円弧と先端矢印を追加で描画する
- 円弧の開始角は固定ではなく、現在の wheel speed ベクトルの向きに合わせる
- `visualization_msgs/msg/MarkerArray` としてパブリッシュする
- RViz では `/visualization/swerve_visualizer` の `MarkerArray` ディスプレイで確認できる

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
| line_width | double | 0.05 | ホイール速度矢印の太さ |
| vector_scale | double | 0.25 | 車輪速度に掛ける表示スケール |
| arrow_color_r | double | 0.1 | ホイール速度矢印の赤成分 |
| arrow_color_g | double | 0.8 | ホイール速度矢印の緑成分 |
| arrow_color_b | double | 0.2 | ホイール速度矢印の青成分 |
| arrow_color_a | double | 1.0 | ホイール速度矢印のアルファ値 |
| show_steering_arc | bool | false | ステア速度の円弧表示を有効にする |
| rotation_vector_line_width | double | line_width * 0.9 | 円弧とステア矢印の線幅 |
| rotation_vector_scale | double | max(line_width * 4.5, 0.36) | 円弧を描く半径 |
| steering_arc_span | double | 0.35 | ステア速度から回転角へ変換する係数 |
| steering_arc_max_span | double | 1.2 | 円弧の最大回転角 |
| steering_arrow_length | double | 0.09 | 円弧先端の矢印長 |
| steering_arrow_head_scale | double | 1.05 | 円弧先端矢印の頭部サイズ係数 |
| steering_arrow_head_cone_scale | double | 1.05 | 円弧先端矢印の先端形状サイズ係数 |
| steering_color_r | double | 1.0 | 円弧とステア矢印の赤成分 |
| steering_color_g | double | 0.55 | 円弧とステア矢印の緑成分 |
| steering_color_b | double | 0.0 | 円弧とステア矢印の青成分 |
| steering_color_a | double | 1.0 | 円弧とステア矢印のアルファ値 |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| marker_array | visualization_msgs/msg/MarkerArray | 変換したステアベクトル |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| command_joint_states | sensor_msgs/msg/JointState | `chassis_calculator` の出力 |
