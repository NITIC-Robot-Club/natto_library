# natto_laser_filter
このパッケージは、LiDARのレーザースキャンデータをフィルタリングするROS2ノードを提供します。

## 機能
- LaserScanデータから影の部分を除去します

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| threshold | double | 0.9 | フィルタリングの閾値(小さいほど除去しやすくなる) |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| output | sensor_msgs::msg::LaserScan | フィルタリングされたLaserScanデータ |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| input | sensor_msgs::msg::LaserScan | 入力となるLaserScanデータ |