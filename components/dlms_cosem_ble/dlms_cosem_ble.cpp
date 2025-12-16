#include "dlms_cosem_ble.h"

#include "esphome/core/log.h"
#include "esphome/core/application.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <sstream>

#include <esp_gatt_defs.h>
#include <esp_heap_caps.h>

#include "dlms_cosem_helpers.h"

#define SET_STATE(st) \
  { \
    ESP_LOGV(TAG, "State change request:"); \
    this->set_next_state_(st); \
  }

static constexpr uint32_t BOOT_TIMEOUT_MS = 10 * 1000;
static constexpr uint32_t SAFEGUARD_INTERVAL_MS = 30 * 1000;
static constexpr uint8_t SAFEGUARD_ERROR_LIMIT = 10;

namespace esphome::dlms_cosem_ble {

static const char *const TAG = "dlms_cosem_ble";

static char empty_str[] = "";

static char format_hex_char(uint8_t v) { return v >= 10 ? 'A' + (v - 10) : '0' + v; }

static ::std::string format_frame_pretty(const uint8_t *data, size_t length) {
  if (length == 0)
    return "";
  ::std::string ret;
  ret.resize(3 * length - 1);
  ::std::ostringstream ss(ret);

  for (size_t i = 0; i < length; i++) {
    switch (data[i]) {
      case 0x00:
        ss << "<NUL>";
        break;
      case 0x01:
        ss << "<SOH>";
        break;
      case 0x02:
        ss << "<STX>";
        break;
      case 0x03:
        ss << "<ETX>";
        break;
      case 0x04:
        ss << "<EOT>";
        break;
      case 0x05:
        ss << "<ENQ>";
        break;
      case 0x06:
        ss << "<ACK>";
        break;
      case 0x0d:
        ss << "<CR>";
        break;
      case 0x0a:
        ss << "<LF>";
        break;
      case 0x15:
        ss << "<NAK>";
        break;
      case 0x20:
        ss << "<SP>";
        break;
      default:
        if (data[i] <= 0x20 || data[i] >= 0x7f) {
          ss << "<" << format_hex_char((data[i] & 0xF0) >> 4) << format_hex_char(data[i] & 0x0F) << ">";
        } else {
          ss << (char) data[i];
        }
        break;
    }
  }
  if (length > 4)
    ss << " (" << length << ")";
  return ss.str();
}

static uint8_t apply_even_parity(uint8_t value) {
  uint8_t bit_count = 0;
  for (uint8_t bit = 0; bit < 7; bit++) {
    bit_count += (value >> bit) & 0x01;
  }
  uint8_t with_msb = value | 0x80;
  if ((bit_count & 0x01) != 0) {
    return with_msb;
  }
  return with_msb & 0x7F;
}

void DlmsCosemBleComponent::InOutBuffers::init(size_t default_in_buf_size) {
  BYTE_BUFFER_INIT(&in);
  bb_capacity(&in, default_in_buf_size);
  mes_init(&out_msg);
  reply_init(&reply);
  this->reset();
}

void DlmsCosemBleComponent::InOutBuffers::reset() {
  mes_clear(&out_msg);
  reply_clear(&reply);
  reply.complete = 1;
  out_msg_index = 0;
  out_msg_data_pos = 0;
  in.size = 0;
  in.position = 0;
  //  amount_in = 0;
}

void DlmsCosemBleComponent::InOutBuffers::check_and_grow_input(uint16_t more_data) {
  const uint16_t GROW_EPSILON = 20;
  if (in.size + more_data > in.capacity) {
    ESP_LOGVV(TAG, "Growing input buffer from %d to %d", in.capacity, in.size + more_data + GROW_EPSILON);
    bb_capacity(&in, in.size + more_data + GROW_EPSILON);
  }
}

void DlmsCosemBleComponent::set_server_address(uint16_t address) { this->server_address_ = address; };

uint16_t DlmsCosemBleComponent::set_server_address(uint16_t logicalAddress, uint16_t physicalAddress,
                                                   unsigned char addressSize) {
  this->server_address_ = cl_getServerAddress(logicalAddress, physicalAddress, addressSize);

  ESP_LOGD(TAG,
           "Server address = %d (based on logical_address=%d, "
           "physical_address=%d, address_size=%d)",
           this->server_address_, logicalAddress, physicalAddress, addressSize);
  return this->server_address_;
}

void DlmsCosemBleComponent::update_server_address(uint16_t addr) {
  this->server_address_ = addr;
  cl_clear(&dlms_settings_);
  cl_init(&dlms_settings_, true, this->client_address_, this->server_address_,
          this->auth_required_ ? DLMS_AUTHENTICATION_LOW : DLMS_AUTHENTICATION_NONE,
          this->auth_required_ ? this->password_.c_str() : NULL, DLMS_INTERFACE_TYPE_HDLC);

  this->update();
}

uint16_t DlmsCosemBleComponent::update_server_address(uint16_t logicalAddress, uint16_t physicalAddress,
                                                      unsigned char addressSize) {
  this->set_server_address(logicalAddress, physicalAddress, addressSize);
  this->update_server_address(this->server_address_);
  return this->server_address_;
}

void DlmsCosemBleComponent::setup() {
  ESP_LOGCONFIG(TAG, "Setting up DLMS/COSEM BLE component");

  if (this->parent_ == nullptr) {
    ESP_LOGE(TAG, "BLE client parent not configured");
    this->mark_failed();
    return;
  }

  if (this->parent_->get_remote_bda() != nullptr) {
    esp_ble_gattc_cache_clean(this->parent_->get_remote_bda());
  }

  cl_init(&dlms_settings_, true, this->client_address_, this->server_address_,
          this->auth_required_ ? DLMS_AUTHENTICATION_LOW : DLMS_AUTHENTICATION_NONE,
          this->auth_required_ ? this->password_.c_str() : NULL, DLMS_INTERFACE_TYPE_HDLC);

  this->buffers_.init(DEFAULT_IN_BUF_SIZE);

  this->set_timeout(BOOT_TIMEOUT_MS, [this]() {
    memset(this->buffers_.in.data, 0, buffers_.in.capacity);
    this->buffers_.in.size = 0;
    this->buffers_.in.position = 0;
    ESP_LOGD(TAG, "Boot timeout, component is ready to use");
    SET_STATE(FsmState::IDLE);
  });

  this->internal_safeguard_();
  ESP_LOGI(TAG, "DLMS/COSEM BLE setup complete.");
}

void DlmsCosemBleComponent::update() { this->try_connect(); }

void DlmsCosemBleComponent::dump_config() {
  ESP_LOGCONFIG(TAG, "DLMS/COSEM BLE Component");
  if (this->parent_ != nullptr) {
    //    ESP_LOGCONFIG(TAG, "  target address: %s", this->parent_->address_str().c_str());
  } else {
    ESP_LOGCONFIG(TAG, "  target address: not set");
  }
  ESP_LOGCONFIG(TAG, "  Supported Meter Types: DLMS/COSEM (SPODES)");
  ESP_LOGCONFIG(TAG, "  Client address: %d", this->client_address_);
  ESP_LOGCONFIG(TAG, "  Server address: %d", this->server_address_);
  ESP_LOGCONFIG(TAG, "  Authentication: %s", this->auth_required_ == DLMS_AUTHENTICATION_NONE ? "None" : "Low");
  ESP_LOGCONFIG(TAG, "  P*ssword: %s", this->password_.c_str());
  ESP_LOGCONFIG(TAG, "  Sensors:");
  for (const auto &sensors : sensors_) {
    auto &s = sensors.second;
    ESP_LOGCONFIG(TAG, "    OBIS code: %s, Name: %s", s->get_obis_code().c_str(), s->get_sensor_name().c_str());
  }
}

void DlmsCosemBleComponent::register_sensor(DlmsCosemBleSensorBase *sensor) {
  this->sensors_.insert({sensor->get_obis_code(), sensor});
}

void DlmsCosemBleComponent::abort_mission_() {
  // Existing pull mode logic
  ESP_LOGE(TAG, "Abort mission. Closing session");
  this->set_next_state_(FsmState::MISSION_FAILED);
}

void DlmsCosemBleComponent::loop() {
  ValueRefsArray vals;                                 // values from brackets, refs to this->buffers_.in
  char *in_param_ptr = (char *) &this->rx_buffer_[1];  // ref to second byte, first is STX/SOH in R1 requests

  switch (this->state_) {
    case FsmState::IDLE: {
    } break;

    case FsmState::BLE_STARTING: {
      if (this->ble_flags_.notifications_enabled && this->ble_flags_.auth_completed) {
        SET_STATE(FsmState::OPEN_SESSION);
      }
    } break;

    case FsmState::COMMS_TX: {
      this->log_state_();
      if (buffers_.has_more_messages_to_send()) {
        send_dlms_messages_();
      } else {
        this->set_next_state_(FsmState::COMMS_RX);
      }
    } break;

    case FsmState::COMMS_RX: {
      this->handle_comms_rx_();
    } break;

    case FsmState::OPEN_SESSION: {
      this->handle_open_session_();
    } break;

    case FsmState::BUFFERS_REQ: {
      this->handle_buffers_req_();
    } break;

    case FsmState::MISSION_FAILED: {
    } break;

    case FsmState::BUFFERS_RCV: {
      this->handle_buffers_rcv_();
    } break;

    case FsmState::ASSOCIATION_REQ: {
      this->handle_association_req_();
    } break;

    case FsmState::ASSOCIATION_RCV: {
      this->handle_association_rcv_();
    } break;

    case FsmState::DATA_RECV: {
      this->handle_data_recv_();
    } break;

    case FsmState::DATA_NEXT: {
      this->handle_data_next_();
    } break;

      // case FsmState::PREPARING_COMMAND: {
      //   // if (this->request_iter == this->sensors_.end()) {
      //   //   SET_STATE(FsmState::PUBLISH);
      //   //   this->parent_->disconnect();
      //   //   break;
      //   // }

      //   // SET_STATE(FsmState::SENDING_COMMAND);

      //   // this->rx_len_ = 0;

      //   // this->ble_flags_.tx_error = false;
      //   // this->ble_flags_.rx_reply = false;

      //   // auto req = this->request_iter->first;
      //   // this->prepare_request_frame_(req.c_str());
      //   // ESP_LOGI(TAG, "Sending request %s (%u bytes payload)", req.c_str(), this->tx_data_remaining_);

      // } break;

      // case FsmState::SENDING_COMMAND: {
      //   // if (this->ble_flags_.tx_error) {
      //   //   ESP_LOGE(TAG, "Error sending data");
      //   //   SET_STATE(FsmState::ERROR);
      //   // } else if (this->tx_data_remaining_ == 0) {
      //   //   SET_STATE(FsmState::READING_RESPONSE);
      //   // }
      // } break;

    case FsmState::READING_RESPONSE: {
      if (this->ble_flags_.rx_reply) {
        SET_STATE(FsmState::GOT_RESPONSE);
      }
    } break;

    case FsmState::GOT_RESPONSE: {
      // do {
      //   if (this->rx_len_ == 0)
      //     break;

      //   auto brackets_found = get_values_from_brackets_(in_param_ptr, vals);
      //   if (!brackets_found)
      //     break;

      //   ESP_LOGD(TAG,
      //            "Received name: '%s', values: %d, idx: 1(%s), 2(%s), 3(%s), "
      //            "4(%s), 5(%s), 6(%s), 7(%s), 8(%s), 9(%s), "
      //            "10(%s), 11(%s), 12(%s)",
      //            in_param_ptr, brackets_found, vals[0], vals[1], vals[2], vals[3], vals[4], vals[5], vals[6],
      //            vals[7], vals[8], vals[9], vals[10], vals[11]);

      //   if (in_param_ptr[0] == '\0') {
      //     if (vals[0][0] == 'E' && vals[0][1] == 'R' && vals[0][2] == 'R') {
      //       ESP_LOGE(TAG, "Request '%s' either not supported or malformed. Error code %s", in_param_ptr, vals[0]);
      //     } else {
      //       ESP_LOGE(TAG, "Request '%s' either not supported or malformed.", in_param_ptr);
      //     }
      //     break;
      //   }

      //   auto req = this->request_iter->first;
      //   if (this->request_iter->second->get_function() != in_param_ptr) {
      //     ESP_LOGE(TAG, "Returned data name mismatch. Skipping frame");
      //     break;
      //   }

      //   auto range = sensors_.equal_range(req);
      //   for (auto it = range.first; it != range.second; ++it) {
      //     if (!it->second->is_failed())
      //       set_sensor_value_(it->second, vals);
      //   }
      // } while (0);

      // this->request_iter = this->sensors_.upper_bound(this->request_iter->first);
      // this->rx_len_ = 0;
      // SET_STATE(FsmState::PREPARING_COMMAND);
    } break;

    case FsmState::PUBLISH: {
      // if (this->sensor_iter != this->sensors_.end()) {
      //   this->sensor_iter->second->publish();
      //   this->sensor_iter++;
      // } else {
      //   if (this->signal_strength_ != nullptr) {
      //     this->signal_strength_->publish_state(this->rssi_);
      //   }
      SET_STATE(FsmState::IDLE);
      //      }
    } break;

    case FsmState::ERROR:
      // do nothing
      break;
  }
}

void DlmsCosemBleComponent::handle_comms_rx_() {
  this->log_state_();

  if (this->check_rx_timeout_()) {
    ESP_LOGE(TAG, "RX timeout.");
    this->has_error = true;
    this->dlms_reading_state_.last_error = DLMS_ERROR_CODE_HARDWARE_FAULT;
    this->stats_.invalid_frames_ += reading_state_.err_invalid_frames;

    if (reading_state_.mission_critical) {
      ESP_LOGE(TAG, "Mission critical RX timeout.");
      this->abort_mission_();
    } else {
      // if not move forward
      reading_state_.err_invalid_frames++;
      this->set_next_state_(reading_state_.next_state);
    }
    return;
  }

  if (!this->ble_flags_.rx_reply) {
    // still no data
    return;
  }

  // the following basic algorithm to be implemented to read DLMS packet
  // first version, no retries
  // 1. receive proper hdlc frame
  // 2. get data from hdlc frame
  // 3. if ret = 0 or ret = DLMS_ERROR_CODE_FALSE then stop
  // 4. check reply->complete. if it is 0 then continue reading, go to 1
  //
  // read hdlc frame
  received_frame_size_ = this->receive_frame_hdlc_();

  if (received_frame_size_ == 0) {
    // keep reading until proper frame is received
    return;
  }

  this->update_last_rx_time_();

  // this->set_next_state_(reading_state_.next_state);

  auto ret = dlms_getData2(&dlms_settings_, &buffers_.in, &buffers_.reply, 0);
  if (ret != DLMS_ERROR_CODE_OK || buffers_.reply.complete == 0) {
    ESP_LOGVV(TAG, "dlms_getData2 ret = %d %s reply.complete = %d", ret, dlms_error_to_string(ret),
              buffers_.reply.complete);
  }

  if (ret != DLMS_ERROR_CODE_OK && ret != DLMS_ERROR_CODE_FALSE) {
    ESP_LOGE(TAG, "dlms_getData2 failed. ret %d %s", ret, dlms_error_to_string(ret));
    this->reading_state_.err_invalid_frames++;
    this->set_next_state_(reading_state_.next_state);
    return;
  }

  if (buffers_.reply.complete == 0) {
    ESP_LOGD(TAG, "DLMS Reply not complete, need more HDLC frames. "
                  "Continue reading.");
    // data in multiple frames.
    // we just keep reading until full reply is received.
    return;  // keep reading
  }

  this->update_last_rx_time_();
  this->set_next_state_(reading_state_.next_state);

  auto parse_ret = this->dlms_reading_state_.parser_fn();
  this->dlms_reading_state_.last_error = parse_ret;

  if (parse_ret == DLMS_ERROR_CODE_OK) {
    //        ESP_LOGD(TAG, "DLSM parser fn result == DLMS_ERROR_CODE_OK");

  } else {
    ESP_LOGE(TAG, "DLMS parser fn error %d %s", parse_ret, dlms_error_to_string(parse_ret));

    if (reading_state_.mission_critical) {
      this->abort_mission_();
    }
    // if not critical - just move forward
    // set_next_state_(FsmState::IDLE);
  }
}

void DlmsCosemBleComponent::handle_open_session_() {
  this->stats_.connections_tried_++;
  this->loop_state_.session_started_ms = millis();
  this->log_state_();
  this->clear_rx_buffers_();
  this->loop_state_.request_iter = this->sensors_.begin();

  this->set_next_state_(FsmState::BUFFERS_REQ);

  // if (false) {
  //   // TODO. check if IEC handshake is needed

  //   uint8_t open_cmd[32]{0};
  //   uint8_t open_cmd_len = snprintf((char *) open_cmd, 32, "/?%s!\r\n",
  //   this->meter_address_.c_str()); request_iter = this->sensors_.begin();
  //   this->send_frame_(open_cmd, open_cmd_len);
  //   this->set_next_state_(FsmState::OPEN_SESSION_GET_ID);
  //   auto read_fn = [this]() { return this->receive_frame_ascii_(); };
  //   // mission crit, no crc
  //   this->read_reply_and_go_next_state_(read_fn,
  //   FsmState::OPEN_SESSION_GET_ID, 0, true, false);
  // }
}

void DlmsCosemBleComponent::handle_buffers_req_() {
  this->log_state_();
  this->prepare_and_send_dlms_buffers();
}

void DlmsCosemBleComponent::handle_buffers_rcv_() {
  this->log_state_();
  // check the reply and go to next stage
  // todo smth with buffers reply
  this->set_next_state_(FsmState::ASSOCIATION_REQ);
}

void DlmsCosemBleComponent::handle_association_req_() {
  this->log_state_();
  // this->start_comms_and_next(&aarq_rr_, FsmState::ASSOCIATION_RCV);
  this->prepare_and_send_dlms_aarq();
}

void DlmsCosemBleComponent::handle_association_rcv_() {
  // check the reply and go to next stage
  // todo smth with aarq reply
  this->set_next_state_(FsmState::DATA_ENQ_UNIT);
}

void DlmsCosemBleComponent::handle_data_enq_() {
  this->log_state_();
  if (this->loop_state_.request_iter == this->sensors_.end()) {
    ESP_LOGD(TAG, "All requests done");
    this->set_next_state_(FsmState::SESSION_RELEASE);
    return;
  }

  auto req = this->loop_state_.request_iter->first;
  auto sens = this->loop_state_.request_iter->second;
  auto type = sens->get_obis_class();
  auto units_were_requested =
      (sens->get_type() == SensorType::SENSOR && type == DLMS_OBJECT_TYPE_REGISTER && !sens->has_got_scale_and_unit());
  if (units_were_requested) {
    auto ret = this->set_sensor_scale_and_unit(static_cast<DlmsCosemBleSensor *>(sens));
  }

  this->buffers_.gx_attribute = 2;
  this->prepare_and_send_dlms_data_request(req.c_str(), type, !units_were_requested);
}

void DlmsCosemBleComponent::handle_data_recv_() {
  this->log_state_();
  this->set_next_state_(FsmState::DATA_NEXT);

  auto req = this->loop_state_.request_iter->first;
  auto sens = this->loop_state_.request_iter->second;
  auto ret = this->set_sensor_value(sens, req.c_str());
}

void DlmsCosemBleComponent::handle_data_next_() {
  this->log_state_();
  this->loop_state_.request_iter = this->sensors_.upper_bound(this->loop_state_.request_iter->first);
  if (this->loop_state_.request_iter != this->sensors_.end()) {
    this->set_next_state_delayed_(this->delay_between_requests_ms_, FsmState::DATA_ENQ_UNIT);
  } else {
    this->set_next_state_delayed_(this->delay_between_requests_ms_, FsmState::SESSION_RELEASE);
  }
}

void DlmsCosemBleComponent::clear_rx_buffers_() {
  memset(this->buffers_.in.data, 0, buffers_.in.capacity);
  this->buffers_.in.size = 0;
  this->buffers_.in.position = 0;
}

size_t DlmsCosemBleComponent::receive_frame_hdlc_() {
  // // HDLC frame: <FLAG>data<FLAG>
  // auto frame_end_check_hdlc = [](uint8_t *b, size_t s) {
  //   auto ret = s >= 2 && b[0] == HDLC_FLAG && b[s - 1] == HDLC_FLAG;
  //   return ret;
  // };
  // return receive_frame_(frame_end_check_hdlc);
  return 0;
}

void DlmsCosemBleComponent::send_dlms_messages_() {
  // this schedules sending from BLE thread
  this->send_next_fragment_();
  
  // // const int MAX_BYTES_IN_ONE_SHOT = 64;
  // gxByteBuffer *buffer = buffers_.out_msg.data[buffers_.out_msg_index];

  // int bytes_to_send = buffer->size - buffers_.out_msg_data_pos;
  // // if (bytes_to_send > 0) {
  // //   if (bytes_to_send > MAX_BYTES_IN_ONE_SHOT)
  // //     bytes_to_send = MAX_BYTES_IN_ONE_SHOT;

  // this->write_array(buffer->data + buffers_.out_msg_data_pos, bytes_to_send);

  // //   ESP_LOGVV(TAG, "TX: %s", format_hex_pretty(buffer->data + buffers_.out_msg_data_pos, bytes_to_send).c_str());

  // //   this->update_last_rx_time_();
  // //   buffers_.out_msg_data_pos += bytes_to_send;
  // // }
  // // if (buffers_.out_msg_data_pos >= buffer->size) {
  // //   buffers_.out_msg_index++;
  // // }
}

// void DlmsCosemBleComponent::prepare_request_frame_(const std::string &request) {
//   //////////////////////////////////////

//   // Prepare the command frame
//   size_t len = snprintf((char *) this->tx_buffer_, TX_BUFFER_SIZE, "/?!\x01R1\x02%s\x03", request.c_str());
//   len++;  // include null terminator

//   // Parity
//   for (size_t i = 0; i < len; i++) {
//     this->tx_buffer_[i] = apply_even_parity(this->tx_buffer_[i]);
//   }

//   this->tx_data_remaining_ = len;
//   this->tx_ptr_ = &this->tx_buffer_[0];

//   this->tx_fragment_started_ = false;
//   this->tx_sequence_counter_ = 0;
// }

void DlmsCosemBleComponent::prepare_and_send_dlms_buffers() {
  auto make = [this]() {
    ESP_LOGD(TAG, "cl_snrmRequest %p ", this->buffers_.out_msg.data);
    return cl_snrmRequest(&this->dlms_settings_, &this->buffers_.out_msg);
  };
  auto parse = [this]() { return cl_parseUAResponse(&this->dlms_settings_, &this->buffers_.reply.data); };
  this->send_dlms_req_and_next(make, parse, FsmState::BUFFERS_RCV, true);
}

void DlmsCosemBleComponent::prepare_and_send_dlms_aarq() {
  auto make = [this]() { return cl_aarqRequest(&this->dlms_settings_, &this->buffers_.out_msg); };
  auto parse = [this]() { return cl_parseAAREResponse(&this->dlms_settings_, &this->buffers_.reply.data); };
  this->send_dlms_req_and_next(make, parse, FsmState::ASSOCIATION_RCV);
}

// void DlmsCosemBleComponent::prepare_and_send_dlms_data_unit_request(const char *obis, int type) {
//   auto ret = cosem_init(BASE(this->buffers_.gx_register), (DLMS_OBJECT_TYPE) type, obis);
//   if (ret != DLMS_ERROR_CODE_OK) {
//     ESP_LOGE(TAG, "cosem_init error %d '%s'", ret, dlms_error_to_string(ret));
//     this->set_next_state_(FsmState::DATA_ENQ);
//     return;
//   }

//   auto make = [this]() {
//     return cl_read(&this->dlms_settings_, BASE(this->buffers_.gx_register), this->buffers_.gx_attribute,
//                    &this->buffers_.out_msg);
//   };
//   auto parse = [this]() {
//     return cl_updateValue(&this->dlms_settings_, BASE(this->buffers_.gx_register), this->buffers_.gx_attribute,
//                           &this->buffers_.reply.dataValue);
//   };
//   this->send_dlms_req_and_next(make, parse, FsmState::DATA_ENQ, false, false);
// }

void DlmsCosemBleComponent::prepare_and_send_dlms_data_request(const char *obis, int type, bool reg_init) {
  int ret = DLMS_ERROR_CODE_OK;
  if (reg_init) {
    ret = cosem_init(BASE(this->buffers_.gx_register), (DLMS_OBJECT_TYPE) type, obis);
  }
  if (ret != DLMS_ERROR_CODE_OK) {
    ESP_LOGE(TAG, "cosem_init error %d '%s'", ret, dlms_error_to_string(ret));
    this->set_next_state_(FsmState::DATA_NEXT);
    return;
  }

  auto make = [this]() {
    return cl_read(&this->dlms_settings_, BASE(this->buffers_.gx_register), this->buffers_.gx_attribute,
                   &this->buffers_.out_msg);
  };
  auto parse = [this]() {
    return cl_updateValue(&this->dlms_settings_, BASE(this->buffers_.gx_register), this->buffers_.gx_attribute,
                          &this->buffers_.reply.dataValue);
  };
  this->send_dlms_req_and_next(make, parse, FsmState::DATA_RECV);
}

void DlmsCosemBleComponent::prepare_and_send_dlms_release() {
  auto make = [this]() { return cl_releaseRequest(&this->dlms_settings_, &this->buffers_.out_msg); };
  auto parse = []() { return DLMS_ERROR_CODE_OK; };
  this->send_dlms_req_and_next(make, parse, FsmState::DISCONNECT_REQ);
}

void DlmsCosemBleComponent::prepare_and_send_dlms_disconnect() {
  auto make = [this]() { return cl_disconnectRequest(&this->dlms_settings_, &this->buffers_.out_msg); };
  auto parse = []() { return DLMS_ERROR_CODE_OK; };
  this->send_dlms_req_and_next(make, parse, FsmState::PUBLISH);
}

void DlmsCosemBleComponent::send_dlms_req_and_next(DlmsRequestMaker maker, DlmsResponseParser parser,
                                                   FsmState next_state, bool mission_critical, bool clear_buffer) {
  dlms_reading_state_.maker_fn = maker;
  dlms_reading_state_.parser_fn = parser;
  dlms_reading_state_.next_state = next_state;
  dlms_reading_state_.mission_critical = mission_critical;
  dlms_reading_state_.reply_is_complete = false;
  dlms_reading_state_.last_error = DLMS_ERROR_CODE_OK;

  // if (clear_buffer) {
  buffers_.reset();
  // }
  int ret = DLMS_ERROR_CODE_OK;
  if (maker != nullptr) {
    ret = maker();
    if (ret != DLMS_ERROR_CODE_OK) {
      ESP_LOGE(TAG, "Error in DLSM request maker function %d '%s'", ret, dlms_error_to_string(ret));
      this->set_next_state_(FsmState::IDLE);
      return;
    }
  }

  reading_state_ = {};
  //  reading_state_.read_fn = read_fn;
  reading_state_.mission_critical = mission_critical;
  reading_state_.tries_max = 1;  // retries;
  reading_state_.tries_counter = 0;
  //  reading_state_.check_crc = check_crc;
  reading_state_.next_state = next_state;
  received_frame_size_ = 0;

  received_complete_reply_ = false;

  this->rx_len_ = 0;

  this->ble_flags_.tx_error = false;
  this->ble_flags_.rx_reply = false;

  this->set_next_state_(FsmState::COMMS_TX);
}

int DlmsCosemBleComponent::set_sensor_scale_and_unit(DlmsCosemBleSensor *sensor) {
  ESP_LOGD(TAG, "set_sensor_scale_and_unit");
  if (!buffers_.reply.complete)
    return DLMS_ERROR_CODE_FALSE;
  auto vt = buffers_.reply.dataType;
  ESP_LOGD(TAG, "DLMS_DATA_TYPE: %s (%d)", dlms_data_type_to_string(vt), vt);
  if (vt != 0) {
    return DLMS_ERROR_CODE_FALSE;
  }

  auto scal = this->buffers_.gx_register.scaler;
  auto unit = this->buffers_.gx_register.unit;
  auto unit_s = obj_getUnitAsString(unit);
  sensor->set_scale_and_unit(scal, unit, unit_s);

  return DLMS_ERROR_CODE_OK;
}

int DlmsCosemBleComponent::set_sensor_value(DlmsCosemBleSensorBase *sensor, const char *obis) {
  if (!buffers_.reply.complete || !sensor->shall_we_publish()) {
    return this->dlms_reading_state_.last_error;
  }

  auto vt = buffers_.reply.dataType;
  auto object_class = sensor->get_obis_class();
  ESP_LOGD(TAG, "Class: %d, OBIS code: %s, DLMS_DATA_TYPE: %s (%d)", object_class, obis, dlms_data_type_to_string(vt),
           vt);

  //      if (cosem_rr_.result().has_value()) {
  if (this->dlms_reading_state_.last_error == DLMS_ERROR_CODE_OK) {
    // result is okay, value shall be there

#ifdef USE_SENSOR
    if (sensor->get_type() == SensorType::SENSOR) {
      if ((object_class == DLMS_OBJECT_TYPE_DATA) || (object_class == DLMS_OBJECT_TYPE_REGISTER) ||
          (object_class == DLMS_OBJECT_TYPE_EXTENDED_REGISTER)) {
        auto var = &this->buffers_.gx_register.value;
        auto scale = static_cast<DlmsCosemBleSensor *>(sensor)->get_scale();
        auto unit = static_cast<DlmsCosemBleSensor *>(sensor)->get_unit();
        if (vt == DLMS_DATA_TYPE_FLOAT32 || vt == DLMS_DATA_TYPE_FLOAT64) {
          float val = var_toDouble(var);
          ESP_LOGD(TAG, "OBIS code: %s, Value: %f, Scale: %f, Unit: %s", obis, val, scale, unit);
          static_cast<DlmsCosemBleSensor *>(sensor)->set_value(val);
        } else {
          int val = var_toInteger(var);
          ESP_LOGD(TAG, "OBIS code: %s, Value: %d, Scale: %f, Unit: %s", obis, val, scale, unit);
          static_cast<DlmsCosemBleSensor *>(sensor)->set_value(val);
        }
      } else {
        ESP_LOGW(TAG, "Wrong OBIS class. Regular numberic sensors can only "
                      "handle Data (class 1), Registers (class = 3) and Extended Registers (class = 4)");
      }
    }
#endif  // USE_SENSOR

#ifdef USE_TEXT_SENSOR
    if (sensor->get_type() == SensorType::TEXT_SENSOR) {
      auto var = &this->buffers_.gx_register.value;
      if (var && var->byteArr && var->byteArr->size > 0) {
        auto arr = var->byteArr;

        ESP_LOGV(TAG, "data size=%d", arr->size);

        bb_setInt8(arr, 0);     // add null-termination
        if (arr->size > 128) {  // clip the string
          ESP_LOGW(TAG, "String is too long %d, clipping to 128 bytes", arr->size);
          arr->data[127] = '\0';
        }
        ESP_LOGV(TAG, "DATA: %s", format_hex_pretty(arr->data, arr->size).c_str());

        if ((object_class == DLMS_OBJECT_TYPE_DATA) || (object_class == DLMS_OBJECT_TYPE_REGISTER) ||
            (object_class == DLMS_OBJECT_TYPE_EXTENDED_REGISTER)) {
          if (vt == DLMS_DATA_TYPE_DATETIME) {
            auto data_as_string = dlms_data_as_string(vt, arr->data, arr->size);
            static_cast<DlmsCosemBleTextSensor *>(sensor)->set_value(data_as_string.c_str(),
                                                                     this->cp1251_conversion_required_);
          } else {
            static_cast<DlmsCosemBleTextSensor *>(sensor)->set_value(reinterpret_cast<const char *>(arr->data),
                                                                     this->cp1251_conversion_required_);
          }
        } else {
          ESP_LOGW(TAG, "Wrong OBIS class. We can only handle Data (class 1), Registers (class = 3) and Extended "
                        "Registers (class = 4)");
        }
      }
    }
#endif
  } else {
    ESP_LOGD(TAG, "OBIS code: %s, result != DLMS_ERROR_CODE_OK = %d", obis, this->dlms_reading_state_.last_error);
  }
  return this->dlms_reading_state_.last_error;
}

bool char2float(const char *str, float &value) {
  char *end;
  value = strtof(str, &end);
  return *end == '\0' || *end == '*' || *end == '#';
}

const LogString *ble_client_state_to_string(espbt::ClientState state) {
  switch (state) {
    case espbt::ClientState::INIT:
      return LOG_STR("INIT");
    case espbt::ClientState::DISCONNECTING:
      return LOG_STR("DISCONNECTING");
    case espbt::ClientState::IDLE:
      return LOG_STR("IDLE");
    case espbt::ClientState::DISCOVERED:
      return LOG_STR("DISCOVERED");
    case espbt::ClientState::CONNECTING:
      return LOG_STR("CONNECTING");
    case espbt::ClientState::CONNECTED:
      return LOG_STR("CONNECTED");
    case espbt::ClientState::ESTABLISHED:
      return LOG_STR("ESTABLISHED");
    default:
      return LOG_STR("UNKNOWN");
  }
}

void DlmsCosemBleComponent::internal_safeguard_() {
  static uint8_t error_states = 0;
  static uint8_t ble_connectings = 0;

  if (this->state_ == FsmState::ERROR) {
    error_states++;
  }

  auto my_ble_state = this->node_state;
  auto parent_ble_state = this->parent_->state();
  if (my_ble_state == espbt::ClientState::CONNECTING) {
    ble_connectings++;
  }

  ESP_LOGV(TAG,
           "Internal safeguard. FSM state is %s, My BLE state is %s, Parent "
           "BLE state is %s. Error states count: %u, BLE connecting states count: %u",
           LOG_STR_ARG(this->state_to_string(this->state_)), LOG_STR_ARG(ble_client_state_to_string(my_ble_state)),
           LOG_STR_ARG(ble_client_state_to_string(parent_ble_state)), error_states, ble_connectings);

  if (error_states > SAFEGUARD_ERROR_LIMIT || ble_connectings > SAFEGUARD_ERROR_LIMIT) {
    ESP_LOGE(TAG, "Too many errors or BLE connecting states. Rebooting.");
    ble_connectings = 0;
    error_states = 0;
    SET_STATE(FsmState::IDLE);
    this->parent_->disconnect();
    delay(100);
    App.reboot();
  }

  this->set_timeout("dlms_cosem_internal_safeguard", SAFEGUARD_INTERVAL_MS, [this]() { this->internal_safeguard_(); });
}

void DlmsCosemBleComponent::remove_bonding() {
  if (this->parent_ == nullptr) {
    return;
  }
  if (this->parent_->get_remote_bda() != nullptr) {
    auto status = esp_ble_remove_bond_device(this->parent_->get_remote_bda());
    ESP_LOGI(TAG, "Bond removal. Status %d", status);
    esp_ble_gattc_cache_clean(this->parent_->get_remote_bda());
  }
  SET_STATE(FsmState::IDLE);
}

void DlmsCosemBleComponent::reset_error() {
  if (this->state_ == FsmState::ERROR) {
    ESP_LOGI(TAG, "Resetting from ERROR state to IDLE");
    SET_STATE(FsmState::IDLE);
  }
}
void DlmsCosemBleComponent::try_connect() {
  if (this->state_ != FsmState::IDLE) {
    ESP_LOGW(TAG, "Not in IDLE state, can't start data collection. Current state is %s",
             LOG_STR_ARG(this->state_to_string(this->state_)));
    return;
  }
  ESP_LOGI(TAG, "Initiating data collection from DLMS/COSEM BLE device");

  this->ble_flags_.raw = 0;
  this->ch_handle_tx_ = 0;
  this->ch_handle_cccd_ = 0;
  this->ch_handle_rx_ = 0;

  this->buffers_.reset();
  // this->tx_fragment_started_ = false;
  // this->tx_sequence_counter_ = 0;
  // this->tx_ptr_ = nullptr;
  // this->tx_data_remaining_ = 0;
  this->rx_len_ = 0;
  this->rx_fragments_expected_ = 0;
  this->rx_current_fragment_ = 0;

  this->loop_state_.request_iter = this->sensors_.begin();
  this->loop_state_.sensor_iter = this->sensors_.begin();

  SET_STATE(FsmState::BLE_STARTING);

  ESP_LOGV(TAG, "Setting desired MTU to %d", DESIRED_MTU);
  esp_ble_gatt_set_local_mtu(DESIRED_MTU);

  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));

  esp_ble_io_cap_t iocap = ESP_IO_CAP_IN;  // ESP_IO_CAP_KBDISP
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));

  uint8_t key_size = 16;
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));

  uint8_t oob_support = ESP_BLE_OOB_DISABLE;
  esp_ble_gap_set_security_param(ESP_BLE_SM_OOB_SUPPORT, &oob_support, sizeof(uint8_t));

  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(init_key));

  uint8_t resp_key = ESP_BLE_ID_KEY_MASK;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &resp_key, sizeof(resp_key));

  this->parent_->set_remote_addr_type(BLE_ADDR_TYPE_RANDOM);  // move to config in future

  this->parent_->connect();
}

void DlmsCosemBleComponent::send_next_fragment_() {
  this->run_in_ble_thread_([this]() {
    // this->ble_flags_.tx_error = ;
    this->ble_send_next_fragment_();
  });
}

void DlmsCosemBleComponent::ble_set_error_() {
  this->parent_->run_later([this]() { this->ble_flags_.tx_error = true; });
}

bool DlmsCosemBleComponent::ble_discover_characteristics_() {
  bool result{true};

  esphome::ble_client::BLECharacteristic *chr;
  ESP_LOGV(TAG, "Discovering DLMS/COSEM characteristics...");
  if (!this->ch_handle_tx_) {
    chr = this->parent_->get_characteristic(this->service_uuid_, this->write_char_uuid_);
    if (chr == nullptr) {
      ESP_LOGW(TAG, "No TX char found");
      result = false;
    } else {
      this->ch_handle_tx_ = chr->handle;
    }
  }

  if (this->ch_handle_rx_ == 0) {
    chr = this->parent_->get_characteristic(this->service_uuid_, this->read_char_uuid_);
    if (chr == nullptr) {
      ESP_LOGW(TAG, "No RX char found");
      result = false;
    } else {
      this->ch_handle_rx_ = chr->handle;
    }
  }

  auto *descr = this->parent_->get_config_descriptor(this->ch_handle_rx_);
  if (descr == nullptr) {
    ESP_LOGW(TAG, "No CCCD descriptor found for RX characteristic");
    result = false;
  } else {
    this->ch_handle_cccd_ = descr->handle;
  }

  return result;
}

uint16_t DlmsCosemBleComponent::get_max_payload_() const {
  if (this->mtu_ <= 4)
    return 0;
  return this->mtu_ - 4;
}

void DlmsCosemBleComponent::run_in_ble_thread_(const ble_defer_fn_t &fn) {
  if (this->parent_ == nullptr) {
    return;
  }
  this->ble_defer_fn_ = fn;
  esp_ble_gap_read_rssi(this->parent_->get_remote_bda());
}

bool DlmsCosemBleComponent::ble_send_next_fragment_() {
  if (this->parent_ == nullptr || this->ch_handle_tx_ == 0) {
    ESP_LOGV(TAG, "BLE component not ready");
    return false;
  }

  uint16_t max_payload = this->get_max_payload_();
  if (max_payload == 0) {
    ESP_LOGW(TAG, "MTU too small to carry command data");
    return false;
  }

  gxByteBuffer *buffer = buffers_.out_msg.data[buffers_.out_msg_index];
  int bytes_to_send = buffer->size - buffers_.out_msg_data_pos;

  if (bytes_to_send <= 0) {
    ESP_LOGV(TAG, "No data to send");
    return false;
  }

  bool more_after_this{false};
  if (bytes_to_send > max_payload) {
    bytes_to_send = max_payload;
    more_after_this = true;
  }

  esp_err_t status = esp_ble_gattc_write_char(
      this->parent_->get_gattc_if(), this->parent_->get_conn_id(), this->ch_handle_tx_, bytes_to_send,
      buffer->data + buffers_.out_msg_data_pos, ESP_GATT_WRITE_TYPE_NO_RSP, ESP_GATT_AUTH_REQ_NONE);
  // this->write_array(buffer->data + buffers_.out_msg_data_pos, bytes_to_send);

  ESP_LOGVV(TAG, "TX: %s", format_hex_pretty(buffer->data + buffers_.out_msg_data_pos, bytes_to_send).c_str());

  // //   this->update_last_rx_time_();
  buffers_.out_msg_data_pos += bytes_to_send;
  // // }
  if (buffers_.out_msg_data_pos >= buffer->size) {
    buffers_.out_msg_index++;
  }

  if (status != ESP_OK) {
    ESP_LOGW(TAG, "esp_ble_gattc_write_char failed: %d", status);
    return false;
  }

  if (more_after_this) {
    // this->tx_ptr_ += payload_len;
    // this->tx_data_remaining_ -= payload_len;
    this->send_next_fragment_();
  } else {
    // this->tx_ptr_ = nullptr;
    // this->tx_data_remaining_ = 0;
  }
  return true;
}

void DlmsCosemBleComponent::ble_initiate_fragment_reads_(uint8_t fragments_to_read) {
  uint8_t fragments = fragments_to_read + 1;

  if (fragments > RX_HANDLES_NUM)
    fragments = RX_HANDLES_NUM;

  this->rx_fragments_expected_ = fragments;
  this->rx_current_fragment_ = 0;
  this->rx_len_ = 0;

  this->ble_request_next_fragment_();
}

void DlmsCosemBleComponent::ble_request_next_fragment_() {
  // Fragmented read path not used for this protocol; notifications carry the full payload.
  this->ble_flags_.rx_reply = true;
  if (this->rx_len_ == 0) {
    return;
  }
  for (size_t i = 0; i < this->rx_len_; i++) {
    this->rx_buffer_[i] = this->rx_buffer_[i] & 0x7F;
  }
  ESP_LOGV(TAG, "RX: %s", format_frame_pretty(this->rx_buffer_, this->rx_len_).c_str());
  ESP_LOGVV(TAG, "RX: %s", format_hex_pretty(this->rx_buffer_, this->rx_len_).c_str());
}

void DlmsCosemBleComponent::ble_read_fragment_(const esp_ble_gattc_cb_param_t::gattc_read_char_evt_param &param) {
  if (param.status != ESP_GATT_OK) {
    ESP_LOGW(TAG, "Response read failed (handle 0x%04X): %d", param.handle, param.status);
    SET_STATE(FsmState::ERROR);
    return;
  }

  if (this->rx_len_ + param.value_len > RX_BUFFER_SIZE)
    return;

  memcpy(this->rx_buffer_ + this->rx_len_, param.value, param.value_len);
  this->rx_len_ += param.value_len;

  this->rx_current_fragment_++;
  this->ble_request_next_fragment_();
}

void DlmsCosemBleComponent::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if,
                                                esp_ble_gattc_cb_param_t *param) {
  switch (event) {
    case ESP_GATTC_OPEN_EVT: {
      if (!this->parent_->check_addr(param->open.remote_bda))
        break;
      if (param->open.status != ESP_GATT_OK) {
        ESP_LOGW(TAG, "Failed to open GATT connection: status=%d", param->open.status);
        SET_STATE(FsmState::ERROR);
      } else {
        esp_ble_gattc_send_mtu_req(this->parent_->get_gattc_if(), this->parent_->get_conn_id());
      }
      break;
    }

    case ESP_GATTC_CFG_MTU_EVT: {
      if (param->cfg_mtu.status == ESP_GATT_OK) {
        this->mtu_ = param->cfg_mtu.mtu;
      }
      break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {
      if (param->search_cmpl.conn_id != this->parent_->get_conn_id())
        break;
      ESP_LOGVV(TAG, "ESP_GATTC_SEARCH_CMPL_EVT: connected=%s, paired=%s", this->parent_->connected() ? "YES" : "NO",
                this->parent_->is_paired() ? "YES" : "NO");

      if (!this->ble_discover_characteristics_()) {
        SET_STATE(FsmState::ERROR);
        this->parent_->disconnect();

        break;
      }

      // TODO: check if characteristics not found - we need to disconnect, then
      //       remove bond, and retry connection to re-establish the bond

      esp_err_t status = esp_ble_gattc_register_for_notify(this->parent_->get_gattc_if(),
                                                           this->parent_->get_remote_bda(), this->ch_handle_rx_);
      if (status != ESP_OK) {
        ESP_LOGW(TAG, "esp_ble_gattc_register_for_notify failed: %d (continuing)", status);
      }

      uint16_t notify_en = 0x0001;
      auto err = esp_ble_gattc_write_char_descr(this->parent_->get_gattc_if(), this->parent_->get_conn_id(),
                                                this->ch_handle_cccd_, sizeof(notify_en), (uint8_t *) &notify_en,
                                                ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE);

      if (err != ESP_OK) {
        ESP_LOGW(TAG, "Failed to enable notifications on handle 0x%04X: %d", this->ch_handle_cccd_, err);
        SET_STATE(FsmState::ERROR);
        return;
      }

    } break;

    case ESP_GATTC_READ_CHAR_EVT: {
      if (param->read.conn_id != this->parent_->get_conn_id())
        break;
      this->ble_read_fragment_(param->read);
      break;
    }

    case ESP_GATTC_WRITE_DESCR_EVT: {
      if (param->write.conn_id != this->parent_->get_conn_id())
        break;
      ESP_LOGVV(TAG, "ESP_GATTC_WRITE_DESCR_EVT received (handle = 0x%04X, status=%d)", param->write.handle,
                param->write.status);

      if (param->write.status == ESP_GATT_OK) {
        this->ble_flags_.notifications_enabled = true;
      } else {
        ESP_LOGW(TAG, "Failed to enable notifications: %d", param->write.status);
        SET_STATE(FsmState::ERROR);
      }
      break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
      if (param->notify.conn_id != this->parent_->get_conn_id())
        break;

      ESP_LOGV(TAG, "Notification received (handle = 0x%04X, )", param->notify.handle, param->notify.value_len);

      if (!param->notify.is_notify)
        break;

      if (!param->notify.value || param->notify.value_len == 0) {
        ESP_LOGW(TAG, "Notification with empty payload received");
        break;
      }

      // if (this->rx_len_ > RX_BUFFER_SIZE)
      //   this->rx_len_ = RX_BUFFER_SIZE;
      this->buffers_.check_and_grow_input(param->notify.value_len);

      auto start_pos = &this->buffers_.in.data[this->buffers_.in.size];
      memcpy(start_pos, param->notify.value, param->notify.value_len);
      this->buffers_.in.size += param->notify.value_len;

      ESP_LOGV(TAG, "RX: %s", format_frame_pretty(start_pos, param->notify.value_len).c_str());
      ESP_LOGVV(TAG, "RX: %s", format_hex_pretty(start_pos, param->notify.value_len).c_str());
      this->ble_flags_.rx_reply = true;
      break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
      if (!this->parent_->check_addr(param->disconnect.remote_bda))
        break;
      ESP_LOGVV(TAG, "GATT client disconnected: reason=0x%02X", param->disconnect.reason);

      this->ch_handle_tx_ = 0;
      this->ch_handle_cccd_ = 0;
      this->ch_handle_rx_ = 0;

      // this->tx_fragment_started_ = false;
      // this->tx_sequence_counter_ = 0;
      this->mtu_ = 23;
      // this->tx_ptr_ = nullptr;
      // this->tx_data_remaining_ = 0;
      this->rx_len_ = 0;
      this->rx_fragments_expected_ = 0;
      this->rx_current_fragment_ = 0;

      if (this->state_ != FsmState::PUBLISH) {
        SET_STATE(FsmState::IDLE);
      }
      break;
    }
    default:
      break;
  }
}

void DlmsCosemBleComponent::gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
  switch (event) {
    case ESP_GAP_BLE_PASSKEY_REQ_EVT: {
      if (!this->parent_->check_addr(param->ble_security.ble_req.bd_addr)) {
        break;
      }
      this->ble_flags_.pin_code_was_requested = true;
      ESP_LOGV(TAG, "Supplying PIN %06u", this->passkey_);
      esp_ble_passkey_reply(param->ble_security.ble_req.bd_addr, true, this->passkey_);
    } break;

    case ESP_GAP_BLE_SEC_REQ_EVT: {
      if (!this->parent_->check_addr(param->ble_security.ble_req.bd_addr)) {
        break;
      }
      auto auth_cmpl = param->ble_security.auth_cmpl;
      ESP_LOGV(TAG, "ESP_GAP_BLE_SEC_REQ_EVT success: %d, fail reason: %d, auth mode: %d", auth_cmpl.success,
               auth_cmpl.fail_reason, auth_cmpl.auth_mode);
      esp_err_t sec_rsp = esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
      ESP_LOGV(TAG, "esp_ble_gap_security_rsp result: %d", sec_rsp);
    } break;

    case ESP_GAP_BLE_AUTH_CMPL_EVT: {
      if (!this->parent_->check_addr(param->ble_security.auth_cmpl.bd_addr)) {
        break;
      }

      if (param->ble_security.auth_cmpl.success) {
        ESP_LOGVV(TAG, "Pairing completed successfully. Did we tell PIN to device ? %s ***",
                  this->ble_flags_.pin_code_was_requested ? "YES" : "NO");
        this->ble_flags_.auth_completed = true;
      } else {
        ESP_LOGE(TAG, "*** Pairing FAILED, reason=%d ***", param->ble_security.auth_cmpl.fail_reason);
      }
    } break;

    case ESP_GAP_BLE_READ_RSSI_COMPLETE_EVT: {
      if (!this->parent_->check_addr(param->read_rssi_cmpl.remote_addr)) {
        break;
      }

      this->rssi_ = param->read_rssi_cmpl.rssi;
      if (this->ble_defer_fn_ != nullptr) {
        this->ble_defer_fn_();
        this->ble_defer_fn_ = nullptr;
      }
    } break;

    default:
      break;
  }
}

/*
    NOT_INITIALIZED = 0,
    IDLE,
    STARTING,

    COMMS_TX,
    COMMS_RX,
    MISSION_FAILED,
    BUFFERS_REQ,
    BUFFERS_RCV,
    ASSOCIATION_REQ,
    ASSOCIATION_RCV,
    DATA_RECV,
    DATA_NEXT,

    PREPARING_COMMAND,
    SENDING_COMMAND,
    READING_RESPONSE,
    GOT_RESPONSE,
    PUBLISH,
    ERROR,
    DISCONNECTED
*/
const LogString *DlmsCosemBleComponent::state_to_string(FsmState state) const {
  switch (state) {
    case FsmState::NOT_INITIALIZED:
      return LOG_STR("NOT_INITIALIZED");
    case FsmState::IDLE:
      return LOG_STR("IDLE");
    case FsmState::BLE_STARTING:
      return LOG_STR("BLE_STARTING");
    case FsmState::COMMS_TX:
      return LOG_STR("COMMS_TX");
    case FsmState::COMMS_RX:
      return LOG_STR("COMMS_RX");
    case FsmState::MISSION_FAILED:
      return LOG_STR("MISSION_FAILED");
    case FsmState::BUFFERS_REQ:
      return LOG_STR("BUFFERS_REQ");
    case FsmState::BUFFERS_RCV:
      return LOG_STR("BUFFERS_RCV");
    case FsmState::ASSOCIATION_REQ:
      return LOG_STR("ASSOCIATION_REQ");
    case FsmState::ASSOCIATION_RCV:
      return LOG_STR("ASSOCIATION_RCV");
    case FsmState::DATA_ENQ:
      return LOG_STR("DATA_ENQ");
    case FsmState::DATA_RECV:
      return LOG_STR("DATA_RECV");
    case FsmState::DATA_NEXT:
      return LOG_STR("DATA_NEXT");

    case FsmState::SENDING_COMMAND:
      return LOG_STR("SENDING_COMMAND");
    case FsmState::READING_RESPONSE:
      return LOG_STR("READING_RESPONSE");
    case FsmState::GOT_RESPONSE:
      return LOG_STR("GOT_RESPONSE");
    case FsmState::PUBLISH:
      return LOG_STR("PUBLISH");
    case FsmState::ERROR:
      return LOG_STR("ERROR");
    case FsmState::DISCONNECTED:
      return LOG_STR("DISCONNECTED");
    default:
      return LOG_STR("UNKNOWN");
  }
}

void DlmsCosemBleComponent::set_next_state_(FsmState state) {
  ESP_LOGV(TAG, "State change: %s -> %s", LOG_STR_ARG(this->state_to_string(this->state_)),
           LOG_STR_ARG(this->state_to_string(state)));
  this->state_ = state;
}

void DlmsCosemBleComponent::set_next_state_delayed_(uint32_t ms, FsmState next_state) {
  if (ms == 0) {
    set_next_state_(next_state);
  } else {
    ESP_LOGV(TAG, "Short delay for %u ms", ms);
    set_next_state_(FsmState::WAIT);
    wait_.start_time = millis();
    wait_.delay_ms = ms;
    wait_.next_state = next_state;
  }
}

void DlmsCosemBleComponent::log_state_(FsmState *next_state) {
  if (this->state_ != this->last_reported_state_) {
    if (next_state == nullptr) {
      ESP_LOGV(TAG, "State::%s", LOG_STR_ARG(state_to_string(this->state_)));
    } else {
      ESP_LOGV(TAG, "State::%s -> %s", LOG_STR_ARG(state_to_string(this->state_)),
               LOG_STR_ARG(state_to_string(*next_state)));
    }
    this->last_reported_state_ = this->state_;
  }
}

}  // namespace esphome::dlms_cosem_ble
