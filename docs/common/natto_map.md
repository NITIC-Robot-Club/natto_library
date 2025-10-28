# natto_map
natto_map パッケージは、natto_msgs/msg/Map メッセージ周りの処理を行います

# map_loader
map_loader ノードは、CSVファイルから地図情報を読み込み、natto_msgs/msg/Map メッセージとしてパブリッシュします。

## 機能
- 指定したCSVファイルから直線セグメントと円弧情報を読み込み
- natto_msgs/msg/Map メッセージとしてパブリッシュ

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| line_segments_path | string | "" | 読み込む直線セグメント (CSV) のファイルパス。空文字の場合は読み込みをスキップします。 |
| circles_path       | string | "" | 読み込む円弧 (CSV) のファイルパス。空文字の場合は読み込みをスキップします。 |
| publish_period_ms  | int    | 1000 | natto_msgs/msg/Map をパブリッシュする周期（ミリ秒） |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| map | natto_msgs/msg/Map | 読み込んだ地図情報 |

## CSVファイルフォーマット
### 直線セグメント (line_segments.csv)
| start_x | start_y | start_z | end_x | end_y | end_z |
| - | - | - | - | - | - |
| 直線セグメントの始点のx座標 (m) | | 直線セグメントの始点のy座標 (m) | 直線セグメントの終点のx座標 (m) | 直線セグメントの終点のy座標 (m) | 直線セグメントの終点のz座標 (m) |

### 円弧 (circles.csv)
| center_x | center_y | center_z | radius | start_angle | end_angle |
| - | - | - | - | - | - |
| 円弧の中心のx座標 (m) | 円弧の中心のy座標 (m) | 円弧の中心のz座標 (m) | 円弧の半径 (m) | 円弧の開始角度 (rad) | 円弧の終了角度 (rad) |

- 角度はx軸正方向を0とし、反時計回りを正とします。

## map creator
[Google SpreadSheetのマップクリエイター](https://docs.google.com/spreadsheets/d/1a26aJcPwLCuzxXE9vYS8q0xv4El7dLMvVLak4jPUAQg/edit?usp=sharing)を使うとかんたんにcsvファイルを作成できます。