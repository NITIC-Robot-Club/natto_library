# natto_msgs
共通で使うメッセージ定義をまとめたパッケージです。

## メッセージ一覧

### Can.msg
| フィールド名 | 型 | 説明 |
| - | - | - |
| header | std_msgs/Header | メッセージヘッダ（タイムスタンプ等） |
| id | uint32 | CAN ID（11ビットまたは29ビット） |
| len | uint8 | データ長コード |
| data | uint8[<=64] | ペイロード（0–64バイト） |
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

### CircleArray.msg
| フィールド名 | 型 | 説明 |
| - | - | - |
| circles | natto_msgs/Circle[] | 複数の円 |

### Line.msg
ax+by+c=0 の形で表される直線のメッセージです。
| フィールド名 | 型 | 説明 |
| - | - | - |
| a | float64 | 直線の係数a |
| b | float64 | 直線の係数b |
| c | float64 | 直線の係数c |

### LineArray.msg
| フィールド名 | 型 | 説明 |
| - | - | - |
| lines | natto_msgs/Line[] | 複数の直線 |

### LineSegment.msg
| フィールド名 | 型 | 説明 |
| - | - | - |
| start | geometry_msgs/Point | 線分の開始点座標 |
| end | geometry_msgs/Point | 線分の終了点座標 |

### LineSegmentArray.msg
| フィールド名 | 型 | 説明 |
| - | - | - |
| line_segments | natto_msgs/LineSegment[] | 複数の線分 |

### Map.msg
| フィールド名 | 型 | 説明 |
| - | - | - |
| line_segments | natto_msgs/LineSegmentArray | 複数の線分 |
| circles | natto_msgs/CircleArray | 複数の円 |

### SpeedPath.msg
| フィールド名 | 型 | 説明 |
| - | - | - |
| header | std_msgs/Header | メッセージヘッダ |
| path | geometry_msgs/PoseStamped[] | 経路の各点（位置・姿勢） |
| twist | geometry_msgs/TwistStamped[] | 各点における速度指令 |

### State.msg
| フィールド名 | 型 | 説明 |
| - | - | - |
| state_id | uint64 | 状態ID |
| state_name | string | 状態名 |
| is_selectable | bool | 選択可能かどうか |

### StateAction.msg
| フィールド名 | 型 | 説明 |
| - | - | - |
| state_id | uint64 | 状態ID |
| action_name | string | アクション名 |
| arguments_names | string[] | 引数名リスト |
| arguments_values | string[] | 引数値リスト |

### StateGraph.msg
| フィールド名 | 型 | 説明 |
| - | - | - |
| graph_name | string | グラフ名 |
| states | natto_msgs/State[] | 状態リスト |
| transitions | natto_msgs/StateTransition[] | 遷移リスト |

### StateResult.msg
| フィールド名 | 型 | 説明 |
| - | - | - |
| state_id | uint64 | 状態ID |
| action_name | string | アクション名 |
| success | bool | 成功フラグ |

### StateTransition.msg
| フィールド名 | 型 | 説明 |
| - | - | - |
| from_state_id | uint64 | 遷移元の状態ID |
| to_state_id | uint64 | 遷移先の状態ID |
| condition | string | 遷移条件 |