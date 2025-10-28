# natto_msgs
共通で使うメッセージ定義をまとめたパッケージです。

## メッセージ一覧

### Can.msg
| フィールド名 | 型 | 説明 |
| - | - | - |
| header | std_msgs/Header | メッセージヘッダ（タイムスタンプ等） |
| id | uint32 | CAN ID（11ビットまたは29ビット） |
| dlc | uint8 | データ長コード |
| data | uint8[64] | ペイロード（0–64バイト） |
| is_error | bool | エラーフラグ |
| is_extended | bool | 拡張フレームフラグ |
| is_rtr | bool | リモート送信要求フラグ |

### Circle.msg
| フィールド名 | 型 | 説明 |
| - | - | - |
| center | geometry_msgs/Point | 円の中心座標 |
| radius | float64 | 円の半径 |
| start_angle | float64 | 円弧の開始角度（ラジアン） |
| end_angle | float64 | 円弧の終了角度（ラジアン） |

### LineSegment.msg
| フィールド名 | 型 | 説明 |
| - | - | - |
| start | geometry_msgs/Point | 線分の開始点座標 |
| end | geometry_msgs/Point | 線分の終了点座標 |

### Map.msg
| フィールド名 | 型 | 説明 |
| - | - | - |
| line_segments | natto_msgs/LineSegment[] | 複数の線分 |
| circles | natto_msgs/Circle[] | 複数の円 |

### Swerve.msg
| フィールド名 | 型 | 説明 |
| - | - | - |
| wheel_angle | float64[] | ホイール角度 |
| wheel_speed | float64[] | ホイール速度(rps) |