
# dlms-cosem-ble = dlms_cosem + ble_nus_client
Нет необходимости в отдельном компоненте для BLE. Используйте связку компонентов [СПОДЭС/DLMS/COSEM](https://github.com/latonita/esphome-dlms-cosem) и [Nordic UART (BLE NUS)](https://github.com/latonita/esphome-nordic-uart-ble).

There is no need for separate BLE component. Use pair of components [СПОДЭС/DLMS/COSEM](https://github.com/latonita/esphome-dlms-cosem) и [Nordic UART (BLE NUS)](https://github.com/latonita/esphome-nordic-uart-ble).


```yaml
external_components:
  - source: github://latonita/esphome-dlms-cosem
    refresh: 10s
    components: [dlms_cosem]
  - source: github://latonita/esphome-nordic-uart-ble
    refresh: 10s
    components: [ble_nus_client]
  
ble_client:
  - mac_address: "11:22:33:44:55:66" # Bluetooth MAC address
    id: nartis_i300_ble
    auto_connect: false
    
esp32_ble_tracker:
  scan_parameters:
    interval: 300ms
    window: 300ms
    active: true    
    continuous: false

ble_nus_client:
  id: ble_uart
  pin: 123456  # PIN code for bluetooth pairing
  service_uuid: 6e400001-b5a3-f393-e0a9-e50e24dc4179
  rx_uuid: 6e400002-b5a3-f393-e0a9-e50e24dc4179
  tx_uuid: 6e400003-b5a3-f393-e0a9-e50e24dc4179   
  mtu: 247
  connect_on_demand: true
  idle_timeout: 5min

dlms_cosem:
  id: nartis_dlms
  uart_id: ble_uart
  client_address: 32
  server_address: 1
  auth: true
  password: "00002080"  # Access password
  receive_timeout: 5000ms
  update_interval: 60s

```
