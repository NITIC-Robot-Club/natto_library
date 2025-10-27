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