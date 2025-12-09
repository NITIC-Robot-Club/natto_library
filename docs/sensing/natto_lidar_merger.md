# natto_lidar_merger
このパッケージは、複数のLiDARデータを統合するROS2ノードを提供します。

# pointcloud2_merger
このノードは、複数の `sensor_msgs/PointCloud2` メッセージを統合し、単一の `sensor_msgs/PointCloud2` メッセージとして出力します。

## 機能
- 複数のLiDARからのPointCloud2データを統合
- ロボットのフットプリント情報を考慮してデータをフィルタリング

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| frame_id | std::string | "merged_pointcloud2_frame" | 統合後のPointCloud2メッセージのフレームID |
| publish_frequency | double | 10.0 | PointCloud2メッセージのパブリッシュ頻度（Hz） |
| lidar_topics | std::vector<std::string> | ["/lidar1/pointcloud2", "/lidar2/pointcloud2"] | 統合するLiDARのPointCloud2トピック名のリスト |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| merged_pointcloud2 | sensor_msgs/PointCloud2 | 統合されたPointCloud2データ |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| footprint | geometry_msgs/PolygonStamped | ロボットのフットプリント情報 |
