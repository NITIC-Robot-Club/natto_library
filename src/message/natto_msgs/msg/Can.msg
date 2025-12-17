std_msgs/Header header
uint32 id         # CAN ID (11 or 29 bit)
uint8 len         # data length code
uint8[<=64] data  # payload (0–64 bytes)
bool is_error     # error flag
bool is_extended  # extended frame flag
bool is_rtr       # remote transmission request flag