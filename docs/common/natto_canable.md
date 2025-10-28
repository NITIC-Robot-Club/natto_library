# natto_canable
CANAbleをSocketCANインターフェースとして使用するためのパッケージです。

# canable
canable ノードは、CANAble デバイスを使用して CAN 通信を行います。
## 機能
- 指定した SocketCAN インターフェース（デフォルト: can0）へソケットを作成してバインド
- 受信データを別スレッドで常時読み取り、natto_msgs::msg::Can 型でパブリッシュ
- natto_msgs::msg::Can 型のトピックを購読して CAN フレームを送信
- 書き込み失敗時のリトライと、必要に応じたソケット再初期化
- 初期化や書き込み処理でのログ出力（INFO/WARN/ERROR/FATAL）

## パラメーター
| パラメーター名 | 型 | デフォルト値 | 説明 |
| - | - | - | - |
| can_interface | string | "can0" | 使用する CAN インターフェース名 |
| retry_open_can | bool | true | CAN ソケット初期化のリトライを有効化 |
| retry_write_can | bool | true | 書き込みリトライを有効化 |
| max_retry_write_count | int | 5 | 書き込みの最大リトライ回数 |

## パブリッシャー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| receive | natto_msgs/msg/Can | 受信した CAN フレーム |

## サブスクライバー
| トピック名 | メッセージ型 | 説明 |
| - | - | - |
| transmit | natto_msgs/msg/Can | 送信する CAN フレーム |

## 事前準備 (candlightの場合)
Classic CAN ~1Mbpsの場合

### ファームウェアの書き込み
1. CANAbleのボタンを押しながらUSBを接続します
2. `USBの場所を確認します
    ```
    $ lsusb
    Bus 001 Device 002: ID 16d0:117e MCS CANable2 b158aa7 github.com/normaldotcom/canable2.git
    ```
    の場合は `/dev/bus/usb/001/002` です
3. 権限を設定します
    ```
    $ sudo chmod 0666 /dev/bus/usb/001/002
    ```
4. [公式サイト](https://canable.io/updater/canable2.html) にアクセスし、`candlelight`を選択し書き込みます

### udevルールの設定
1. CANAbleのシリアル番号を確認します
    ```
    $ lsusb
    Bus 001 Device 003: ID 1d50:606f OpenMoko, Inc. Geschwister Schneider CAN adapter
    ```
    ```
    $ lsusb -v -d 1d50:606f | grep Serial
    iSerial                 3 004A00433136500C2039384D
    ```
    この場合`004A00433136500C2039384D`がシリアル番号になります

2. `/etc/udev/rules.d/99-canable.rules` を作成し、以下の内容を記述します
    ```
    SUBSYSTEM=="usb", ATTR{idVendor}=="1d50", ATTR{idProduct}=="606f", ATTR{serial}=="004A00433136500C2039384D", SYMLINK+="can", MODE="0666", \
    RUN+="/sbin/ip link set can0 type can bitrate 1000000", \
    RUN+="/sbin/ip link set can0 up"
    ```

    bitrateは使用するCANの通信速度に合わせて変更してください

3. udevルールを再読み込みします
    ```
    $ sudo udevadm control --reload-rules
    $ sudo udevadm trigger
    ```
4. CANAbleを接続し、CANとして認識しているかを確認します
    ```
    $ ls /dev/can
    /dev/can
    ```
    または
    ```
    $ ip a
    41: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP group default qlen 10
    link/can 
    ```

## 事前準備 (slcanの場合)
FD CAN ~5Mbpsの場合やWindowsでデバッグしながら使いたい場合

### ファームウェアの書き込み
1. CANAbleのボタンを押しながらUSBを接続します
2. `USBの場所を確認します
    ```
    $ lsusb
    Bus 001 Device 002: ID 16d0:117e MCS CANable2 b158aa7 github.com/normaldotcom/canable2.git
    ```
    の場合は `/dev/bus/usb/001/002` です
3. 権限を設定します
    ```
    $ sudo chmod 0666 /dev/bus/usb/001/002
    ```
4. [公式サイト](https://canable.io/updater/canable2.html) にアクセスし、`candlelight`を選択し書き込みます


### udevルールの設定
1. CANAbleのシリアル番号を確認します
    ```
    $ lsusb
    Bus 001 Device 003: ID 1d50:606f OpenMoko, Inc. Geschwister Schneider CAN adapter
    ```
    ```
    $ lsusb -v -d 1d50:606f | grep Serial
    iSerial                 3 208338903136
    ```
    この場合`208338903136`がシリアル番号になります

2. ここから先はslcanでやるが、udevで自動化したら何故かうまく行かなかったのであとでやってみる
    ```bash
    slcand -o -c -s8 /dev/ttyACM0 can0
    sudo ip link set can0 up
    sudo ip link set can0 txqueuelen 1000
    ```

## 使用方法
1. CANの接続を確認します
    ```bash
    candump can0
    ```
    大量の通信ログが流れてきたら成功です
2. natto_canableノードを起動します
    ```bash
    ros2 run natto_canable canable
    ```
3. CANフレームの送受信を行います
    ```bash
    ros2 topic echo /can/receive
    ```
    ```bash
    ros2 topic pub /can/transmit natto_msgs/msg/Can "{id: 0x123, dlc: 8, data: [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08]}"
    ```