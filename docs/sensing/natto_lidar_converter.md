# natto_lidar_converter
このパッケージは、LiDARデータの形式変換を行うROS2ノードを提供します。

# laserscan_to_pointcloud2
このノードは、`sensor_msgs/LaserScan` メッセージを `sensor_msgs/PointCloud2` メッセージに変換します。

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| frame_id | std::string | "pointcloud2_frame" | 変換後のPointCloud2メッセージのフレームID |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| /pointcloud2 | sensor_msgs::msg::PointCloud2 | 変換されたPointCloud2データ |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| /laser_scan | sensor_msgs::msg::LaserScan | 入力となるLaserScanデータ |


# pointcloud2_to_laserscan
このノードは、`sensor_msgs/PointCloud2` メッセージを `sensor_msgs/LaserScan` メッセージに変換します。

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| frame_id | std::string | "pointcloud2_frame" | 入力となる
| angle_increment | double | 0.0174533 (約1度) | 生成されるLaserScanの角度分解能（rad） |
| range_min | double | 0.0 | 生成されるLaserScanの最小測定距離（m） |
| range_max | double | 30.0 |  生成されるLaserScanの最大測定距離（m） |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| /laser_scan | sensor_msgs::msg::LaserScan | 変換されたLaserScanデータ |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| /pointcloud2 | sensor_msgs::msg::PointCloud2 | 入力となるPointCloud2データ |
