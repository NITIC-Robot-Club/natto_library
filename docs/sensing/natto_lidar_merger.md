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
| frame_id | string | "pointcloud2_frame" | 変換先フレームID |
| frequency | double | 40.0 | publish周期（Hz） |
| mode | string | "merge" | `merge` は統合して1本にする。`passthrough` は各LiDARを個別に変換・フットプリント除去して同一トピックへ流す。 |
| lidar_topics | string[] | ["/lidar1_pointcloud2", "/lidar2_pointcloud2"] | 処理対象のLiDAR PointCloud2トピック名のリスト |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| merged_pointcloud2 | sensor_msgs/PointCloud2 | 統合されたPointCloud2データ |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| footprint | geometry_msgs/PolygonStamped | ロボットのフットプリント情報 |
