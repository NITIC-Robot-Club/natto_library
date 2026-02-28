# natto_chassis_calculator
natto_chassis_calculator パッケージは、ロボットの走行制御に関する計算ノードを提供します。オムニホイール、メカナムホイール、独立ステアリングの計算をサポートします。

# chassis_calculator
chassis_calculator ノードは、ロボットの指令速度から各ホイールの角度と速度を計算します。

## 機能
- geometry_msgs/msg/TwistStamped メッセージを受信
- chassis_typeに応じて各ホイールの角度と速度を計算
- sensor_msgs/msg/JointState メッセージとしてパブリッシュ
- URDFからホイールの位置と角度を動的に取得

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| chassis_type | string | "" | シャーシのタイプ（"swerve", "omni", "mecanum"） |
| wheel_radius | double | 0.05 | ホイールの半径（m） |
| wheel_names | std::vector<string> | {""} | ホイールのジョイント名のリスト |
| wheel_base_names | std::vector<string> | {""} | ホイールベースのジョイント名のリスト |
| infinite_swerve_mode | bool | false | 独ステ用：無限回転モードを有効にするかどうか |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| command_joint_states | sensor_msgs/msg/JointState | 計算されたホイールの目標角度と速度 |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| command_velocity | geometry_msgs/msg/TwistStamped | ロボットの指令速度 |
| joint_states | sensor_msgs/msg/JointState | 現在のジョイント状態（独ステ用） |

## 対応シャーシタイプ

### swerve (独立ステアリング)
4輪独立ステアリング機構のロボット。各ホイールが独立して回転方向を変更できます。

### omni (オムニホイール)
オムニホイール機構のロボット。ホイールの取り付け角度に応じて全方向移動が可能です。

### mecanum (メカナムホイール)
メカナムホイール機構のロボット。オムニホイールと同様の計算式で、ホイールの取り付け角度に基づいて速度を計算します。

## 使用例

### URDFの設定
\`\`\`xml
<joint name="mecanum_wheel_0_base" type="fixed">
  <parent link="base_link"/>
  <child link="mecanum_wheel_0_base_link"/>
  <origin xyz="0.45 0.45 0.05" rpy="0 0 -0.7854"/>  <!-- -45度 -->
</joint>

<joint name="mecanum_wheel_0" type="continuous">
  <parent link="mecanum_wheel_0_base_link"/>
  <child link="mecanum_wheel_0_link"/>
  <axis xyz="0 1 0"/>
</joint>
\`\`\`

### パラメータファイル
\`\`\`yaml
/**:
  ros__parameters:
    chassis_type: "mecanum"
    wheel_radius: 0.05
    wheel_names: ["mecanum_wheel_0", "mecanum_wheel_1", "mecanum_wheel_2", "mecanum_wheel_3"]
    wheel_base_names: ["mecanum_wheel_0_base", "mecanum_wheel_1_base", "mecanum_wheel_2_base", "mecanum_wheel_3_base"]
\`\`\`
