# natto_ransac
このパッケージは、RANSACアルゴリズムを用いて点群データから形状を検出するROS2ノードを提供します。

# line_detector
line_detectorノードは、RANSACアルゴリズムを使用して `sensor_msgs/PointCloud2` メッセージから線分を検出し、検出された線分とコーナーをそれぞれ `natto_msgs/LineSegmentArray` および `geometry_msgs/PoseArray` メッセージとして出力します。

## 機能
- 点群データからRANSACアルゴリズムを用いて直線を検出
- 検出された直線を線分に分割
- 検出された直線からコーナーを抽出

## パラメータ
| パラメータ名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| max_iterations | int | 100 | RANSACの最大反復回数 |
| max_lines | int | 10 | 検出する最大線分数 |
| min_inliers | int | 75 | 線分として認識するための最小インライア数 |
| distance_threshold | double | 0.01 | RANSAC直線の太さ |
| segment_gap_threshold | double | 0.1 | 線分を分割するためのギャップ閾値 |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| lines | natto_msgs/LineArray | 検出された線分の配列 |
| line_segments | natto_msgs/LineSegmentArray | 検出された線分の配列 |
| corners | geometry_msgs/PoseArray | 検出されたコーナーの配列 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| pointcloud2 | sensor_msgs/PointCloud2 | 入力点群データ |
