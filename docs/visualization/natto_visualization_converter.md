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
