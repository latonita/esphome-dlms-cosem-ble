#pragma once

#include "esphome/components/ble_client/ble_client.h"
#include "esphome/core/component.h"

#ifdef USE_TIME
#include "esphome/components/time/real_time_clock.h"
#endif

#include <esp_gattc_api.h>

#include <array>
#include <map>
#include <string>
#include <vector>

#include "dlms_cosem_ble_sensor.h"

#include <client.h>
#include <converters.h>
#include <cosem.h>
#include <dlmssettings.h>

namespace espbt = esphome::esp32_ble_tracker;
namespace esphome {
namespace dlms_cosem_ble {

static const size_t DEFAULT_IN_BUF_SIZE = 256;

const uint8_t VAL_NUM = 12;
using ValueRefsArray = ::std::array<char *, VAL_NUM>;
using SensorMap = ::std::multimap<::std::string, DlmsCosemBleSensorBase *>;

using ble_defer_fn_t = ::std::function<void()>;  // NOLINT

using FrameStopFunction = std::function<bool(uint8_t *buf, size_t size)>;
using ReadFunction = std::function<size_t()>;

using DlmsRequestMaker = std::function<int()>;
using DlmsResponseParser = std::function<int()>;

static constexpr size_t RX_HANDLES_NUM = 15;
static constexpr size_t TX_BUFFER_SIZE = 64;
static constexpr size_t RX_BUFFER_SIZE = 256;

static constexpr size_t DESIRED_MTU = 247;

class DlmsCosemBleComponent : public PollingComponent, public ble_client::BLEClientNode {
 public:
  DlmsCosemBleComponent(){};

  void setup() override;
  void loop() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::AFTER_BLUETOOTH; };

  // BLE configuration
  void set_passkey(uint32_t passkey) { this->passkey_ = passkey % 1000000U; };
  void set_service_uuid(const char *uuid_str) { this->service_uuid_ = espbt::ESPBTUUID::from_raw(uuid_str); }
  void set_write_char_uuid(const char *uuid_str) { this->write_char_uuid_ = espbt::ESPBTUUID::from_raw(uuid_str); }
  void set_read_char_uuid(const char *uuid_str) { this->read_char_uuid_ = espbt::ESPBTUUID::from_raw(uuid_str); }

  // DLMS configuration
  void set_client_address(uint16_t addr) { this->client_address_ = addr; };

  void set_server_address(uint16_t addr);
  uint16_t set_server_address(uint16_t logicalAddress, uint16_t physicalAddress, unsigned char addressSize);
  void update_server_address(uint16_t addr);
  uint16_t update_server_address(uint16_t logicalAddress, uint16_t physicalAddress, unsigned char addressSize);

  void set_auth_required(bool auth) { this->auth_required_ = auth; };
  void set_password(const std::string &addr) { this->password_ = addr; };
  void set_cp1251_conversion_required(bool required) { this->cp1251_conversion_required_ = required; }
  void set_receive_timeout_ms(uint32_t timeout) { this->receive_timeout_ms_ = timeout; };

  // Sensors
  void set_signal_strength(sensor::Sensor *signal_strength) { signal_strength_ = signal_strength; }

  // OBIS sensors
  void register_sensor(DlmsCosemBleSensorBase *sensor);

  // Lambda actions
  void try_connect();
  void remove_bonding();
  void reset_error();

 protected:
  bool set_sensor_value_(DlmsCosemBleSensorBase *sensor, ValueRefsArray &vals);

  enum class FsmState : uint8_t {
    NOT_INITIALIZED = 0,
    IDLE,
    BLE_STARTING,
    WAIT,

    COMMS_TX,
    COMMS_RX,
    MISSION_FAILED,

    OPEN_SESSION,
    BUFFERS_REQ,
    BUFFERS_RCV,
    ASSOCIATION_REQ,
    ASSOCIATION_RCV,
    DATA_ENQ_UNIT,
    DATA_ENQ,
    DATA_RECV,
    DATA_NEXT,

    SESSION_RELEASE,
    DISCONNECT_REQ,

    PUBLISH,
    ERROR,
    DISCONNECTED
  };
  FsmState state_{FsmState::NOT_INITIALIZED};
  FsmState last_reported_state_{FsmState::NOT_INITIALIZED};

  const LogString *state_to_string(FsmState state) const;
  void log_state_(FsmState *next_state = nullptr);

  void set_next_state_(FsmState state);
  void set_next_state_delayed_(uint32_t delay_ms, FsmState next_state);

  //
  // BLE related functions
  // 
  void send_next_ble_fragment_();

  // BLE send/receive
  uint16_t get_max_ble_payload_() const;
  void run_in_ble_thread_(const ble_defer_fn_t &fn);
  ble_defer_fn_t ble_defer_fn_{nullptr};

  // BLE Thread functions
  bool ble_discover_characteristics_();
  void ble_set_error_();
  bool ble_send_next_fragment_();
  void ble_initiate_fragment_reads_(uint8_t slots);
  void ble_request_next_fragment_();
  void ble_read_fragment_(const esp_ble_gattc_cb_param_t::gattc_read_char_evt_param &param);

  void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                           esp_ble_gattc_cb_param_t *param) override;
  void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) override;

  uint16_t mtu_{23};

  union {
    uint8_t raw;
    struct {
      bool notifications_enabled : 1;
      bool pin_code_was_requested : 1;
      bool auth_completed : 1;
      bool tx_error : 1;

      bool rx_reply : 1;
      bool tx_started : 1;
      bool tx_completed : 1;
      bool tx_more_after : 1;
    };
  } ble_flags_{0};

  // service, write char, read char

  espbt::ESPBTUUID service_uuid_;
  espbt::ESPBTUUID write_char_uuid_;
  espbt::ESPBTUUID read_char_uuid_;

  // Handles
  uint16_t ch_version_{0};
  uint16_t ch_handle_tx_{0};
  uint16_t ch_handle_cccd_{0};
  uint16_t ch_handle_rx_{0};

  int8_t rssi_{0};
  uint32_t passkey_{0};

  sensor::Sensor *signal_strength_{nullptr};
  void internal_safeguard_();

  // end of BLE part


  // DLMS/COSEM functions
  void prepare_and_send_dlms_buffers();
  void prepare_and_send_dlms_aarq();
  void prepare_and_send_dlms_auth();
  void prepare_and_send_dlms_data_unit_request(const char *obis, int obis_class);
  void prepare_and_send_dlms_data_request(const char *obis, int obis_class, bool reg_init = true);
  void prepare_and_send_dlms_release();
  void prepare_and_send_dlms_disconnect();
  void send_dlms_req_and_next(DlmsRequestMaker maker, DlmsResponseParser parser, FsmState next_state,
                              bool mission_critical = false, bool clear_buffer = true);

  // State handler methods extracted from loop()
  void handle_comms_tx_();
  void handle_comms_rx_();
  void handle_open_session_();
  void handle_buffers_req_();
  void handle_buffers_rcv_();
  void handle_association_req_();
  void handle_association_rcv_();
  void handle_data_enq_unit_();
  void handle_data_enq_();
  void handle_data_recv_();
  void handle_data_next_();
  void handle_session_release_();
  void handle_disconnect_req_();
  void handle_publish_();

  int set_sensor_scale_and_unit(DlmsCosemBleSensor *sensor);
  int set_sensor_value(DlmsCosemBleSensorBase *sensor, const char *obis);

  bool skip_unit_request_{false};

  struct {
    ReadFunction read_fn;
    FsmState next_state;
    bool mission_critical;
    bool check_crc;
    uint8_t tries_max;
    uint8_t tries_counter;
    uint32_t err_crc;
    uint32_t err_invalid_frames;
  } reading_state_{nullptr, FsmState::IDLE, false, false, 0, 0, 0, 0};
  size_t received_frame_size_{0};
  bool received_complete_reply_{false};

  struct {
    DlmsRequestMaker maker_fn;
    DlmsResponseParser parser_fn;
    FsmState next_state;
    bool mission_critical;
    bool reply_is_complete;
    int last_error;
  } dlms_reading_state_{nullptr, nullptr, FsmState::IDLE, false, false, DLMS_ERROR_CODE_OK};
  uint32_t last_rx_time_{0};
  struct LoopState {
    uint32_t session_started_ms{0};             // start of session
    SensorMap::iterator request_iter{nullptr};  // talking to meter
    SensorMap::iterator sensor_iter{nullptr};   // publishing sensor values

  } loop_state_;

  struct InOutBuffers {
    message out_msg;
    uint16_t out_msg_index{0};
    uint16_t out_msg_data_pos{0};
    gxByteBuffer in;
    size_t in_position;

    gxReplyData reply;

    void init(size_t default_in_buf_size);
    void reset();
    void check_and_grow_input(uint16_t more_data);
    // next function shows whether there are still messages to send
    bool has_more_messages_to_send() const { return out_msg_index < out_msg.size; }

    gxRegister gx_register;
    gxClock gx_clock;
    unsigned char gx_attribute{2};

  } buffers_;

 protected:
  dlmsSettings dlms_settings_;
  uint16_t client_address_{16};
  uint16_t server_address_{1};
  bool auth_required_{false};
  std::string password_{""};
  uint32_t receive_timeout_ms_{2000};
  uint32_t delay_between_requests_ms_{0};
  bool cp1251_conversion_required_{true};
  struct {
    uint32_t start_time{0};
    uint32_t delay_ms{0};
    FsmState next_state{FsmState::IDLE};
  } wait_;
  SensorMap sensors_{};

  void clear_rx_buffers_();

  void queue_dlms_messages_();
  size_t receive_frame_(FrameStopFunction stop_fn);

  size_t receive_frame_hdlc_();

  size_t receive_frame_raw_();
  uint32_t time_raw_limit_{0};

  inline void update_last_rx_time_() { this->last_rx_time_ = millis(); }
  bool check_wait_timeout_() { return millis() - wait_.start_time >= wait_.delay_ms; }
  bool check_rx_timeout_() { return millis() - this->last_rx_time_ >= receive_timeout_ms_; }

  void abort_mission_();

  struct Stats {
    uint32_t connections_tried_{0};
    uint32_t crc_errors_{0};
    uint32_t crc_errors_recovered_{0};
    uint32_t invalid_frames_{0};
    uint8_t failures_{0};

    float crc_errors_per_session() const { return (float) crc_errors_ / connections_tried_; }
  } stats_;
  void stats_dump_();
  bool has_error{true};
};

}  // namespace dlms_cosem_ble
}  // namespace esphome
