#pragma once

#include "esphome/components/uart/uart.h"
#include "esphome/core/component.h"
#include "esphome/components/alarm_control_panel/alarm_control_panel.h"

#define TEST_TIMER(timer, duration) ((((uint32_t) millis()) - timer) > duration)
#define NEXT_QUEUE_INDEX(prev, depth) ((prev + 1 >= depth) ? 0 : prev + 1)

namespace esphome {
namespace kpa1 {

const char CODE_START_OVER_KEY = '*';
const uint8_t CODE_DIGITS_MAX = 4;
const uint8_t MAX_KEYPAD_LINE = 16;
const uint8_t MAX_KEYPAD_DATA_LENGTH = 16;
const uint8_t MAX_ASSISTANCE_KEY = 4;
const uint8_t RAW_PACKET_BUFFER_SIZE = 64;
const uint8_t TX_DATA_PACKET_POOL_SIZE = 8;
const uint8_t KDU_POOL_SIZE = 8;
const uint8_t KP_MODEL_LEN = 6;
const uint8_t KP_INFO_MAX_KEYPADS = 8;
const uint8_t STUFF_CODE = 0x00;
const uint8_t SOH = 0x01;
const uint8_t STX = 0x02;
const uint8_t ETX = 0x03;
const uint8_t STUFFED_BYTE_THRESHOLD = 0x04;
const uint8_t PT_ACK = 0x00;
const uint8_t PT_NAK = 0x01;
const uint8_t PT_DATA_SHORT = 0x02;
const uint16_t CRC_INIT_VEC = 0x55AA;
const uint8_t PANEL_MAX_RETRIES = 5;
const uint32_t PACKET_TX_TIMEOUT_MS = 2000;
const uint32_t RX_FRAME_TIMEOUT_MS = 2000;
const uint32_t CODE_INTERDIGIT_TIMEOUT_MS = 5000;
const uint32_t POWER_ON_WAIT_MS = 10000;
const uint32_t BACKLIGHT_TIME_MS = 60000;
const uint32_t KEYPAD_UPDATE_TIME = 500;
const uint32_t READY_LED_UPDATE_TIME_MS = 1000;
const uint32_t REMOTE_ERROR_COUNTER_UPDATE_TIME_MS = 60000;
const uint32_t COMMAND_VALID_FLAG_TIME_MS = 1000;
const uint32_t HELLO_BACKOFF_TIME_MS = 30000;
const uint32_t CHIME_UPDATE_TIME_MS = 5000;

/*
 * Enums used in communication with the panel
 */

enum {
  CHIME_NONE = 0,
  CHIME_ONCE,
  CHIME_TWICE,
  CHIME_THREE_TIMES,
  CHIME_FAST_REPEATING,
  CHIME_SLOW_REPEATING,
  CHIME_UNUSED,
  CHIME_LOUD
};
enum { KEYPAD_RECORD_TYPE_RESERVED = 0, KEYPAD_RECORD_KEYS };
enum {
  RTYPE_HELLO = 0,
  RTYPE_SEND_ERROR_COUNTERS,
  RTYPE_UPDATE_KEYPAD,
  RTYPE_DATA_FROM_KEYPAD,
  RTYPE_ECHO,
  RTYPE_CONN_KEYPADS,
  RTYPE_VERSION,
};

enum { CR_IDLE = 0, CR_BUSY };
enum {
  PSF_CLEAR = 0x00,
  PSF_RX_ACK = 0x01,
  PSF_RX_NAK = 0x02,
  PSF_RX_DATA = 0x04,
  PSF_BAD_PACKET = 0x08,
  PSF_INIT = 0x40,
  PSF_TX_BUSY = 0x80,
  PSF_RX_FLAGS = 0x0F
};
enum { RX_GOT_NOTHING = 0, RX_GOT_STX, RX_GOT_ETX, RX_GOT_DATA };
enum { PRX_STATE_INIT = 0, PRX_STATE_IDLE, PRX_TX, PRX_TX_WAIT_ACK, PRX_HELLO_BACKOFF };
enum { RF_STATE_IDLE = 0, RF_WAIT_DATA_ETX, RF_WAIT_CLEAR_FLAGS };
enum { SRX_STATE_IDLE = 0, SRX_STATE_WAIT_SECOND };

/*
 * Local structs not used outside of this component
 */

using ErrorCountersLocal = struct alignas(1) ErrorCountersLocal {
  uint32_t tx_soft_errors;
  uint32_t tx_hard_errors;
  uint32_t tx_buffer_pool_overflow_errors;
  uint32_t rx_bad_packets;
  uint32_t rx_frame_timeouts;
  uint32_t pad[3];
};

/*
 * Structs used in communication with the kpa1
 */
using VersionInfo = struct alignas(1) VersionInfo {
  uint8_t device_id;
  uint8_t version_major;
  uint8_t version_mid;
  uint8_t version_minor;
};

using PanelPacketAckNak = struct alignas(1) PanelPacketAckNak {
  uint8_t type;
  uint8_t seq_num;
  uint8_t crc16_l;
  uint8_t crc16_h;
};

using PanelPacketHeader = struct alignas(1) PanelPacketHeader {
  uint8_t type;
  uint8_t seq_num;
  uint8_t payload_len;
};

using RecordTypeHeader = struct alignas(1) RecordTypeHeader {
  uint8_t record_type;
  uint8_t data_length;
};

using KeypadDisplayUpdate = struct alignas(1) KeypadDisplayUpdate {
  bool ready;
  bool armed;
  bool back_light;
  uint8_t keypad_address;
  uint8_t chime;
  uint8_t len_line1;
  uint8_t len_line2;
  uint8_t line1[MAX_KEYPAD_LINE];
  uint8_t line2[MAX_KEYPAD_LINE];

};

using PanelKeyboardEvent =  struct alignas(1) PanelKeyboardEvent {
  uint8_t record_type;
  uint8_t keypad_address;
  uint8_t action;
  uint8_t record_data_length;
  uint8_t record_data[MAX_KEYPAD_DATA_LENGTH];
};

using PanelKeypadType = struct alignas(1) PanelKeypadType {
  uint8_t valid;
  uint8_t model[KP_MODEL_LEN];
};

using PanelKeypadInfo = struct alignas(1) PanelKeypadInfo {
  PanelKeypadType info[KP_INFO_MAX_KEYPADS];
};

using ErrorCountersRemote = struct alignas(1) ErrorCountersRemote {
  uint32_t tx_soft_errors;
  uint32_t tx_hard_errors;
  uint32_t tx_buffer_pool_overflow_errors;
  uint32_t rx_bad_packets;
  uint32_t rx_frame_timeouts;
  uint32_t ecp_parity_errors;
  uint32_t ecp_checksum_errors;
  uint32_t pad;
};

using EchoCommand = struct alignas(1) EchoCommand {
  uint8_t length;
  uint8_t data[8];
};

class Kpa1 : public uart::UARTDevice, public Component {
 protected:
  alarm_control_panel::AlarmControlPanel *acp_;
  ErrorCountersLocal ec_local_;
  ErrorCountersRemote ec_remote_;
  PanelKeypadInfo ki_;
  VersionInfo vi_;
  PanelPacketAckNak txAckNakPacket_;
  KeypadDisplayUpdate queuedKdu_;
  KeypadDisplayUpdate dequeuedKdu_;
  KeypadDisplayUpdate kduPool_[KDU_POOL_SIZE];
  bool assistanceKey_[MAX_ASSISTANCE_KEY];
  bool keypadBacklightState_;
  bool kpa1Hello_;
  bool helloReceived_;
  bool fastReadyLed_;
  bool keypadEntrySilent_;
  bool keypadExitSilent_;
  bool keypadAlarmSilent_;
  bool codeAndCommandReceived_;
  bool commProblem_;
  uint8_t txDataDequeuedPacket_[RAW_PACKET_BUFFER_SIZE];
  uint8_t txDataQueuedPacket_[RAW_PACKET_BUFFER_SIZE];
  uint8_t rxDataPacket_[RAW_PACKET_BUFFER_SIZE];
  uint8_t keypadDigitsReceived_[CODE_DIGITS_MAX + 1];
  uint8_t alarmState_;
  uint8_t codeReceiverState_;
  uint8_t codeInProcess_[CODE_DIGITS_MAX + 1];
  uint8_t keypadDigitSourceAddress_;
  uint8_t keypadDigitCount_;
  uint8_t keypadCommand_;
  uint8_t codeDigitCount_;
  uint8_t codeDigitSourceAddress_;
  uint8_t codeCommandDigit_;
  uint8_t rxAckPacketSequenceNumber_;
  uint8_t rxDataPacketSequenceNumber_;
  uint8_t kduHead_;
  uint8_t kduTail_;
  uint8_t txDataPoolHead_;
  uint8_t txDataPoolTail_;
  uint8_t txPacketPool_[TX_DATA_PACKET_POOL_SIZE][RAW_PACKET_BUFFER_SIZE];
  uint8_t stuffedRxState_;
  uint8_t rxFrameState_;
  uint8_t rxFrameIndex_;
  uint8_t packetState_;
  uint8_t packetStateFlags_;
  uint8_t txSeqNum_;
  uint8_t lastRxSeqNum_;
  uint8_t txRetries_;
  uint32_t powerOnTimer_;
  uint32_t rxFrameTimer_;
  uint32_t txTimer_;
  uint32_t backLightTimer_;
  uint32_t keypadUpdateTimer_;
  uint32_t codeReceiverTimer_;
  uint32_t readyLedTimer_;
  uint32_t remoteErrorCounterTimer_;
  uint32_t validCommandTimer_;
  uint32_t helloBackoffTimer_;
  uint32_t chimeTimer_;

  void logDebugHex_(const char *desc, void *p, uint32_t length);
  uint16_t crc16_(const uint8_t *data, uint16_t len, uint16_t crc, uint16_t poly = 0x1021, bool refin = false,
                  bool refout = false);
  void makeTxDataPacket_(uint8_t *packet, uint8_t record_type, void *data = NULL);
  void makeTxAckNakPacket_(uint8_t data_type, uint8_t seq_num);
  bool queueTxPacket_(void *tx_packet_in);
  bool deQueueTxPacket_(void *tx_packet_out);
  bool validateRxPacket_(uint8_t data_len, void *data, uint8_t *packet_type);
  void stuffedTx_(uint8_t tx_byte);
  int stuffedRx_(uint8_t *rx_byte);
  void rxFrame_();
  void commStateMachineHandler_();
  void keypadUpdateHandler_();
  void receiveCodeDigitsHandler_();
  void txFrame_(void *tx_packet_in);
  void processKeypadPresentMessage_(PanelKeyboardEvent *pke);
  void processKeypadKeyPresses_(PanelKeyboardEvent *pke);
  void processDataPacket_();
  void sendCommandToController_(uint8_t keypad_address, char command, char *code);
  void backlightOn_();
  bool kduEnqueue_(KeypadDisplayUpdate *kdu);
  bool kduDequeue_(KeypadDisplayUpdate *kdu);
  void backlightHandler_();
  void readyLedHandler_();
  void lcdCopyString_(int line, int pos, const char *text);
  void remoteErrorCountersHandler_();
  void processRemoteErrorCounters_(ErrorCountersRemote *prec);
  void processKeypadInfo_(PanelKeypadInfo *prec);
  bool getAssistanceKey_(uint8_t key);

 public:
  Kpa1();

  void setup() override;

  void loop() override;

  void dump_config() override;

  //
  // Set alarm control panel object
  // Called by esphome. Do not call
  //

  void set_acp(alarm_control_panel::AlarmControlPanel *acp_id);

  //
  // Configures keypad silence entry state
  // Called by esphome, do not call
  //

  void set_keypad_entry_silent(bool silent);

  //
  // Configures keypad silence exit state
  // Called by esphome, do not call
  //

  void set_keypad_exit_silent(bool silent);

  //
  // Configures keypad silence alarm state
  // Called by esphome, do not call
  //

  void set_keypad_alarm_silent(bool silent);

  //
  // The YAML code should call this when the alarm panel state changes
  // status - the state value from the alarm panel component
  //

  void update_alarm_state(uint8_t status);

  //
  // Update system ready state
  // The yaml code should call this whenever the system ready state changes
  // and after initial power on
  //

  void update_system_ready(bool ready);

  //
  // Send entry chime
  // The yaml code should call this whenever a chime zone goes from closed to open
  //

  void send_entry_chime();

  //
  //
  // Check for zone faults. Return true if any of the zone state bits
  // are true after masking, or false if all zone bits after masking are false
  //
  // To mask a zone, set the appropriate mask bit to zero in the zone_bits mask.
  //

  bool check_for_zone_faults(uint64_t zone_state_bits, uint64_t zone_bits_mask);

  //
  // Dumps all error counters to the log
  //

  void dump_error_counters();

  //
  // Returns true if there's a communication problem with the keypads.
  //

  bool get_keypad_comm_problem();

  //
  // Return number of keypads connected
  //

  uint8_t get_keypad_count();

  //
  // Return the keypad data as a c string in the following format:
  //
  // model@aa,model@aa,....
  //
  // Where: aa is the keypad address in decimal
  // and model is the keypad model number.

  const char *get_keypad_info();

  //
  // Return the assistance key value. Returns true if one was pressed.
  // Return false if no assistance key was pressed.
  // After a true is returned, the assistance key value returned will
  // be zero on subsequent calls until a new key is pressed.
  //

  bool get_assistance_key_a();  // Topmost

  bool get_assistance_key_b();

  bool get_assistance_key_c();

  bool get_assistance_key_d();  // Bottom
};

}  // namespace kpa1
}  // namespace esphome
