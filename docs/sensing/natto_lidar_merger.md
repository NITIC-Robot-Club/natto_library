# natto_lidar_merger
このパッケージは、複数のLiDARデータを統合するROS2ノードを提供します。

# pointcloud2_merger
このノードは、複数の `sensor_msgs/PointCloud2` メッセージを統合し、単一の `sensor_msgs/PointCloud2` メッセージとして出力します。

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| frame_id | std::string | "merged_pointcloud2_frame" | 統合後のPointCloud2メッセージのフレームID |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| merged_pointcloud2 | sensor_msgs/PointCloud2 | 統合されたPointCloud2データ |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| pointcloud2_first | sensor_msgs/PointCloud2 | 統合する最初のPointCloud2データ |
| pointcloud2_second | sensor_msgs/PointCloud2 | 統合する二番目のPointCloud2データ |
| footprint | geometry_msgs/PolygonStamped | ロボットのフットプリント情報 |
