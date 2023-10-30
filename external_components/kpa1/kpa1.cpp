#include "kpa1.h"
#include "esphome/core/log.h"


#define ESP_LL1 ESP_LOGD
#define ESP_LL2 ESP_LOGD

namespace esphome {
namespace kpa1 {
  
static const char *TAG = "kpa1";

// Customizable text messages

const char *NOT_READY_TEXT = "Not Ready";
const char *READY_TO_ARM_TEXT = "Ready to Arm";
const char *DISARMED_TEXT = "Disarmed";
const char *ARMED_HOME_TEXT ="Armed Stay";
const char *ARMED_AWAY_TEXT = "Armed Away";
const char *ARMED_NIGHT_TEXT = "Armed Night";
const char *ARMED_VACATION_TEXT = "Armed Vacation";
const char *ARMED_CUSTOM_BYPASS_TEXT = "Armed Cust. Byp.";
const char *ALARM_PENDING_TEXT = "Entry Delay";
const char *ENTER_CODE_NOW_TEXT = "Enter Code Now";
const char *ALARM_ARMING_TEXT = "Exit Delay";
const char *LEAVE_NOW_TEXT = "Leave Now";
const char *DISARMING_TEXT = "Disarming...";
const char *TRIGGERED_TEXT = "*** ALARM ***";


Kpa1::Kpa1() {
  // These are in the constructor, because the setters for them are called
  // before begin() is called.
  this->keypadAlarmSilent_ = this->keypadEntrySilent_ = this->keypadExitSilent_ = false;
}

/*
 * Dump hex helper function
 */

void Kpa1::logDebugHex_(const char *desc, void *p, uint32_t length) {
  char hex_string[16 * 3 + 1];
  uint32_t lines;
  uint32_t len;

  if (length > 16) {
    lines = (length / 16);
    if (length % 16) {
      lines++;
    }
  } else {
    lines = 1;
  }

  ESP_LL1(TAG, "%s", desc);
  for (uint32_t line = 0; line < lines; line++) {
    if (length > 16) {
      len = 16;
      length -= 16;
    } else {
      len = length;
    }
    for (uint32_t i = 0; i < len; i++) {
      snprintf(hex_string + (i * 3), 4, "%02X ", ((uint8_t *) p)[i + (line * 16)]);
    }
    ESP_LL1(TAG, "%s", hex_string);
  }
}

/*
 * Copy string to the indicated lcd line and position
 */

void Kpa1::lcdCopyString_(int line, int pos, const char *text) {
  int len = strlen(text);
  // Sanity checks
  if (len > MAX_KEYPAD_LINE) {
    ESP_LOGW(TAG, "Display line  length clipped");
    len = MAX_KEYPAD_LINE;
  }
  if (len - pos < 0) {
    ESP_LOGE(TAG, "Bug: len - pos < 0");
    return;
  }

  char *pl;

  if (line == 0) {
    pl = (char *) this->queuedKdu_.line1;
    this->queuedKdu_.len_line1 = MAX_KEYPAD_LINE;

  } else {
    pl = (char *) this->queuedKdu_.line2;
    this->queuedKdu_.len_line2 = MAX_KEYPAD_LINE;
  }

  // Transfer text
  for (int i = 0; i < len; i++)
    pl[i + pos] = text[i];

  // Pad the line out to the last display character
  for (int i = pos + len; i < MAX_KEYPAD_LINE; i++) {
    pl[i] = ' ';
  }
}

/*
 * Transmit a byte with octet stuffing
 */

void Kpa1::stuffedTx_(uint8_t tx_byte) {
  if (tx_byte < STUFFED_BYTE_THRESHOLD) {
    write_byte(STUFF_CODE);
  }
  write_byte(tx_byte);
}

/*
 * Receive a byte with octet stuffing
 */

int Kpa1::stuffedRx_(uint8_t *rx_byte) {
  switch (this->stuffedRxState_) {
    case SRX_STATE_IDLE:
      if (available()) {
        read_byte(rx_byte);
        if (*rx_byte == STUFF_CODE) {
          this->stuffedRxState_ = SRX_STATE_WAIT_SECOND;
        } else {
          if (*rx_byte == STX) {
            return RX_GOT_STX;
          } else if (*rx_byte == ETX) {
            return RX_GOT_ETX;
          } else if (*rx_byte == SOH) {  // Unused SOH control byte
            break;
          } else {
            return RX_GOT_DATA;
          }
        }
      }
      break;

    case SRX_STATE_WAIT_SECOND:
      if (available()) {
        read_byte(rx_byte);
        this->stuffedRxState_ = SRX_STATE_IDLE;
        return RX_GOT_DATA;
      }
      break;

    default:
      this->stuffedRxState_ = SRX_STATE_IDLE;
      break;
  }
  return RX_GOT_NOTHING;
}

/*
 * Calculate CRC for data passed in
 */

uint16_t Kpa1::crc16_(const uint8_t *data, uint16_t len, uint16_t crc, uint16_t poly, bool refin, bool refout) {
  // Use ESPHome helper function
  return esphome::crc16be(data, len, crc, poly, refin, refout);
}

/*
 * Make a data packet to be queued in the TX pool
 */

void Kpa1::makeTxDataPacket_(uint8_t *packet, uint8_t record_type, void *data) {
  uint8_t clipped_data_len;
  uint8_t payload_bytes_remaining;
  PanelPacketHeader *p = (PanelPacketHeader *) packet;
  PanelKeyboardEvent *pke;
  ESP_LL1(TAG, "Making data packet: rtype = %d", record_type);
  // Record type preprocessing and validation
  switch (record_type) {
    case RTYPE_HELLO:
      clipped_data_len = 0;
      break;

    case RTYPE_SEND_ERROR_COUNTERS:
      clipped_data_len = 0;
      break;

    case RTYPE_UPDATE_KEYPAD:
      clipped_data_len = sizeof(KeypadDisplayUpdate);
      break;

    case RTYPE_ECHO:
      clipped_data_len = sizeof(EchoCommand);
      break;

    case RTYPE_CONN_KEYPADS:
      clipped_data_len = 0;
      break;
      
    case RTYPE_VERSION:
      clipped_data_len = 0;
      break;

    default:
      return;  // Don't know what the record type is
  }

  // Point to record type header
  RecordTypeHeader *rth = (RecordTypeHeader *) (packet + sizeof(PanelPacketHeader));

  // Point to data start
  uint8_t *data_start = packet + (sizeof(PanelPacketHeader) + sizeof(RecordTypeHeader));

  // Point to CRC start
  uint8_t *crc_start = packet + ((sizeof(PanelPacketHeader) + sizeof(RecordTypeHeader) + clipped_data_len));

  // Set header fields
  p->type = PT_DATA_SHORT;
  p->seq_num = this->txSeqNum_++;
  p->payload_len = clipped_data_len + sizeof(RecordTypeHeader);
  // Set the record type and length
  rth->record_type = record_type;
  rth->data_length = clipped_data_len;

  // Copy the data, if any
  if ((clipped_data_len != 0) && (data != NULL)) {
    memcpy(data_start, data, clipped_data_len);
  }
  // Calculate CRC
  uint16_t crc =
      crc16_(packet, (clipped_data_len + sizeof(PanelPacketHeader) + sizeof(RecordTypeHeader)), CRC_INIT_VEC);
  // Insert CRC
  crc_start[0] = (uint8_t) crc;
  crc_start[1] = (uint8_t) (crc >> 8);
}

/*
 * Make an ACK or NAK packet
 */

void Kpa1::makeTxAckNakPacket_(uint8_t data_type, uint8_t seq_num) {
  // Set header fields
  if (data_type == 0) {
    ESP_LL1(TAG, "Making ACK packet");
  } else {
    ESP_LL1(TAG, "Making NAK packet");
  }

  this->txAckNakPacket_.type = data_type;
  this->txAckNakPacket_.seq_num = seq_num;
  // Calculate CRC
  uint16_t crc = crc16_((uint8_t *) &this->txAckNakPacket_, 2, CRC_INIT_VEC);
  // Insert CRC
  this->txAckNakPacket_.crc16_l = (uint8_t) crc;
  this->txAckNakPacket_.crc16_h = (uint8_t) (crc >> 8);
}

/*
 * Validate a packet received
 */

bool Kpa1::validateRxPacket_(uint8_t data_len, void *data, uint8_t *packet_type) {
  PanelPacketAckNak *a = (PanelPacketAckNak *) data;
  uint16_t crc;

  switch (a->type) {
    case PT_ACK:
    case PT_NAK:
      crc = crc16_((uint8_t *) data, 2, CRC_INIT_VEC);
      if (((crc & 0xFF) == a->crc16_l) && ((crc >> 8) == a->crc16_h)) {
        *packet_type = a->type;
        return true;
      }
      break;

    case PT_DATA_SHORT: {
      // Calculate the CRC
      crc = crc16_((uint8_t *) data, data_len - 2, CRC_INIT_VEC);
      // Calculate CRC location in the buffer
      uint8_t *crc_start = ((uint8_t *) data) + (data_len - 2);
      // Check CRC
      if (((crc & 0xFF) == crc_start[0]) && ((crc >> 8) == crc_start[1])) {
        *packet_type = a->type;
        return true;
      }
      break;
    }

    default:
      break;
  }
  return false;
}

/*
 * Receive a frame
 */

void Kpa1::rxFrame_() {
  uint8_t rx_byte;
  int res;
  uint8_t pt;

  switch (this->rxFrameState_) {
    case RF_STATE_IDLE:
      res = stuffedRx_(&rx_byte);
      if (res == RX_GOT_STX) {
        this->rxFrameTimer_ = millis();
        this->rxFrameIndex_ = 0;
        this->rxFrameState_ = RF_WAIT_DATA_ETX;
      }
      break;

    case RF_WAIT_DATA_ETX:
      if (TEST_TIMER(this->rxFrameTimer_, RX_FRAME_TIMEOUT_MS)) {
        // We timed out, start over
        this->ec_local_.rx_frame_timeouts++;
        this->rxFrameState_ = RF_STATE_IDLE;
      }
      res = stuffedRx_(&rx_byte);
      if (res != RX_GOT_NOTHING) {
        if (res == RX_GOT_ETX) {  // End of frame
          if (validateRxPacket_(this->rxFrameIndex_, this->rxDataPacket_, &pt)) {
            ESP_LL1(TAG, "Got frame type: %d, sequence number: %d, time (ms): %d", this->rxDataPacket_[0],
                    this->rxDataPacket_[1], millis());
            // Got a packet, set the packet state flags accordingly
            PanelPacketAckNak *ppan = (PanelPacketAckNak *) this->rxDataPacket_;
            if (pt == PT_ACK) {
              this->rxAckPacketSequenceNumber_ = ppan->seq_num;
              this->packetStateFlags_ |= PSF_RX_ACK;
              this->rxFrameState_ = RF_WAIT_CLEAR_FLAGS;
            } else if (pt == PT_NAK) {
              this->rxAckPacketSequenceNumber_ = ppan->seq_num;
              this->packetStateFlags_ |= PSF_RX_NAK;
              this->rxFrameState_ = RF_WAIT_CLEAR_FLAGS;
            } else if (pt == PT_DATA_SHORT) {
              // Save the data packet sequence number for subsequent ACK'ing.
              this->rxDataPacketSequenceNumber_ =
                  ppan->seq_num;  // Note: In the same position as an ACK/NAK packet data structure
              this->packetStateFlags_ |= PSF_RX_DATA;
              this->rxFrameState_ = RF_WAIT_CLEAR_FLAGS;
            } else {
              // Got something we don't understand
              this->rxFrameState_ = RF_STATE_IDLE;
            }
          } else {
            // Packet Validation failed
            this->packetStateFlags_ |= PSF_BAD_PACKET;
            this->rxFrameState_ = RF_STATE_IDLE;
          }
        } else if (res == RX_GOT_DATA) {  // Data byte
          this->rxDataPacket_[this->rxFrameIndex_++] = rx_byte;
        } else {  // Unexpected frame control byte
          this->rxFrameState_ = RF_STATE_IDLE;
        }
      }
      break;

    case RF_WAIT_CLEAR_FLAGS:
      // Wait for main state machine to process the frame
      // before attempting to receive another.
      if ((this->packetStateFlags_ & PSF_RX_FLAGS) == 0)
        this->rxFrameState_ = RF_STATE_IDLE;
      break;

    default:
      this->rxFrameState_ = RF_STATE_IDLE;
      break;
  }
}

/*
 * Transmit a frame
 */

void Kpa1::txFrame_(void *tx_packet_in) {
  PanelPacketHeader *h = (PanelPacketHeader *) tx_packet_in;
  PanelPacketAckNak *a = (PanelPacketAckNak *) tx_packet_in;
  uint8_t *p = (uint8_t *) tx_packet_in;
  uint8_t tx_length;

  if (a->type == PT_DATA_SHORT) {
    tx_length = sizeof(PanelPacketHeader) + sizeof(uint16_t) +
                h->payload_len;  // Get total packet length (3 bytes of header plus 2 bytes of CRC)
    // logDebugHex_("TX packet contents:", tx_packet_in, tx_length);
  } else if ((a->type == PT_ACK) || (a->type == PT_NAK)) {
    tx_length = sizeof(PanelPacketAckNak);
  } else {
    return;  // Unknown packet type
  }
  // Send the packet
  write_byte(STX);
  for (int i = 0; i < tx_length; i++) {
    // Transmit the packet
    stuffedTx_(p[i]);
  }
  write_byte(ETX);
  flush();
  ESP_LL1(TAG, "Frame transmitted, type: %d, sequence number : %d, time (ms): %d", p[0], p[1], millis());
}

/*
 * Enqueue packet for later transmission
 */

bool Kpa1::queueTxPacket_(void *tx_packet_in) {
  uint8_t next_head = NEXT_QUEUE_INDEX(this->txDataPoolHead_, TX_DATA_PACKET_POOL_SIZE);
  // If pool is full, return false
  if (next_head == this->txDataPoolTail_) {
    ESP_LOGW(TAG, "TX packet queue full, discarding message");
    return false;
  }
  // Determine how much to transfer
  PanelPacketHeader *h = (PanelPacketHeader *) tx_packet_in;
  uint8_t byte_count = h->payload_len + sizeof(PanelPacketHeader) + 2;  // 2 bytes for CRC

  // Copy packet into pool
  memcpy(txPacketPool_[this->txDataPoolHead_], tx_packet_in, byte_count);
  this->txDataPoolHead_ = next_head;
  return true;
}

/*
 * Dequeue packet for transmission
 */

bool Kpa1::deQueueTxPacket_(void *tx_packet_out) {
  // If pool is empty, return false
  if (this->txDataPoolHead_ == this->txDataPoolTail_) {
    return false;
  }
  uint8_t next_tail = NEXT_QUEUE_INDEX(this->txDataPoolTail_, TX_DATA_PACKET_POOL_SIZE);

  // Determine how much to transfer
  PanelPacketHeader *h = (PanelPacketHeader *) txPacketPool_[this->txDataPoolTail_];
  RecordTypeHeader *rth = (RecordTypeHeader *) (txPacketPool_[this->txDataPoolTail_] + sizeof(PanelPacketHeader));
  uint8_t byte_count = h->payload_len + sizeof(PanelPacketHeader) + 2;  // 2 bytes for CRC
  ESP_LL2(TAG, "Dequeing packet with record type %d", rth->record_type);

  // Copy packet out of pool
  memcpy(tx_packet_out, txPacketPool_[this->txDataPoolTail_], byte_count);
  this->txDataPoolTail_ = next_tail;
  return true;
}

/*
 * Queue a Keypad Display Update message
 */

bool Kpa1::kduEnqueue_(KeypadDisplayUpdate *kdu) {
  uint8_t next_head = NEXT_QUEUE_INDEX(this->kduHead_, KDU_POOL_SIZE);
  if (next_head == this->kduTail_) {
    ESP_LOGW(TAG, "Keypad update queue full, discarding message");
    return false;
  }
  ESP_LL1(TAG, "Queing keypad display update");
  memcpy(&kduPool_[this->kduHead_], kdu, sizeof(KeypadDisplayUpdate));
  this->kduHead_ = next_head;
  return true;
}

/*
 * Dequeue a Keypad Display Update message from the pool
 */

bool Kpa1::kduDequeue_(KeypadDisplayUpdate *kdu) {
  uint8_t next_tail = NEXT_QUEUE_INDEX(this->kduTail_, KDU_POOL_SIZE);
  if (this->kduTail_ == this->kduHead_) {
    return false;
  }
  ESP_LL1(TAG, "Dequeing keypad display update");
  memcpy(kdu, &kduPool_[this->kduTail_], sizeof(KeypadDisplayUpdate));
  this->kduTail_ = next_tail;
  return true;
}

/*
 *
 * Process keypad presses sent by keypad
 * Makes a local copy in the object of the keypad data
 * for use by receiveCodeDigits_()
 *
 */

void Kpa1::processKeypadKeyPresses_(PanelKeyboardEvent *pke) {
  if (this->keypadDigitCount_ == 0) {  // Only copy if the previous data has been processed
    this->keypadDigitSourceAddress_ = pke->keypad_address;
    this->keypadDigitCount_ = pke->record_data_length;
    if (this->keypadDigitCount_ > CODE_DIGITS_MAX + 1) {
      this->keypadDigitCount_ = CODE_DIGITS_MAX + 1;
    }
    // Copy and convert key codes to ASCII
    for (int i = 0; i < this->keypadDigitCount_; i++) {
      uint8_t c = pke->record_data[i];
      if (c <= 9) {  // 0-9 case
        c = c + 0x30;
      } else if (c == 0x0A) {
        c = '*';
      } else if (c == 0x0B) {
        c = '#';
      } else if ((c >= 0x1C) && (c <= 0x1F)) {
        c += ('A' - 0x1C);
      } else {
        ESP_LOGW(TAG, "Unexpected key code: %02X, ignoring", c);
        continue;
      }
      keypadDigitsReceived_[i] = c;
    }
  } else {
    ESP_LOGW(TAG, "Code digit buffer in use, digits ignored from other keypad");
  }
}

/*
 * Process upload of the error counters from the kpa1
 */

void Kpa1::processRemoteErrorCounters_(ErrorCountersRemote *prec) {
  ESP_LL1(TAG, "Remote error counters received");
  memcpy(&ec_remote_, prec, sizeof(ErrorCountersRemote));
}

/*
 * Process upload of keypad info from the kpa1
 */

void Kpa1::processKeypadInfo_(PanelKeypadInfo *prec) {
  memcpy(&ki_, prec, sizeof(PanelKeypadInfo));
  ESP_LL1(TAG, "Keypad information received");
  // logDebugHex_("Keypad information:",&ki_, sizeof(PanelKeypadInfo));
}

/*
 * Process a data packet received from the kpa1. This will normally be
 * 1 or more keypad digits entered by the user, but at power on,
 * additional messages are sent so these are handled here as well.
 */

void Kpa1::processDataPacket_() {
  PanelPacketHeader *pph = (PanelPacketHeader *) this->rxDataPacket_;
  RecordTypeHeader *rth = (RecordTypeHeader *) (this->rxDataPacket_ + sizeof(PanelPacketHeader));

  switch (rth->record_type) {
    case RTYPE_HELLO:
      ESP_LOGI(TAG, "HELLO received from KPA1");
      this->kpa1Hello_ = true;  // Flag to unlock the TX queue in ACK when it is received.
      break;

    case RTYPE_ECHO:
      break;

    case RTYPE_SEND_ERROR_COUNTERS: {
      ErrorCountersRemote *prec =
          (ErrorCountersRemote *) (this->rxDataPacket_ + sizeof(PanelPacketHeader) + sizeof(RecordTypeHeader));
      processRemoteErrorCounters_(prec);
      break;
    }

    case RTYPE_CONN_KEYPADS: {
      PanelKeypadInfo *prec =
          (PanelKeypadInfo *) (this->rxDataPacket_ + sizeof(PanelPacketHeader) + sizeof(RecordTypeHeader));
      processKeypadInfo_(prec);
      break;
    }

    case RTYPE_DATA_FROM_KEYPAD: {
      // Determine what command we have and call the appropriate processing function
      PanelKeyboardEvent *pke =
          (PanelKeyboardEvent *) (this->rxDataPacket_ + sizeof(PanelPacketHeader) + sizeof(RecordTypeHeader));
      if (pke->record_type == KEYPAD_RECORD_KEYS) {
        processKeypadKeyPresses_(pke);
      }
      break;
    }
    
    case RTYPE_VERSION: {
      VersionInfo *prec = (VersionInfo *) (this->rxDataPacket_ + sizeof(PanelPacketHeader) + sizeof(RecordTypeHeader));
      memcpy(&this->vi_, prec, sizeof(VersionInfo));
      ESP_LOGI(TAG, "KPA1 device id: %02d, major version: %02d, mid version: %02d, minor version: %02d",
      vi_.device_id, vi_.version_major, vi_.version_mid, vi_.version_minor);
      break;
    }

    default:
      break;
  }
}

/*
 * Handle communcications to and from the kpa1
 */

void Kpa1::commStateMachineHandler_() {
  uint32_t now = millis();

  switch (this->packetState_) {
    case PRX_STATE_INIT: {
      if (TEST_TIMER(this->powerOnTimer_, POWER_ON_WAIT_MS)) {
        // Send hello packet on initial power on after power on wait time
        ESP_LOGI(TAG, "Sending HELLO to the KPA1");
        makeTxDataPacket_(this->txDataDequeuedPacket_, RTYPE_HELLO);
        this->packetStateFlags_ = (PSF_TX_BUSY | PSF_INIT);
        this->txRetries_ = 0;
        this->packetState_ = PRX_TX;
      }
      break;
    }
    case PRX_STATE_IDLE: {
      // Data case
      if (this->packetStateFlags_ & PSF_RX_DATA) {
        // We received a data packet
        ESP_LL1(TAG, "Received data packet. Sequence number: %d, record type: %d", this->rxDataPacket_[1],
                this->rxDataPacket_[3]);
        // logDebugHex_("Received bytes", this->rxDataPacket_, this->rxDataPacket_[2] + sizeof(PanelPacketHeader));  //
        // DEBUG
        //  Handle received packet
        processDataPacket_();

        // Make ACK Data packet
        makeTxAckNakPacket_(PT_ACK, this->rxDataPacketSequenceNumber_);

        // Transmit it
        txFrame_(&this->txAckNakPacket_);
        ESP_LL1(TAG, "ACK/NAK packet transmitted");

        // This unlocks the TX queue once we have seen an ACK to the KPA1's HELLO message
        if ((!this->helloReceived_) && this->kpa1Hello_) {
          ESP_LL1(TAG, "Unlocking the TX queue");
          this->helloReceived_ = this->kpa1Hello_;
          this->kpa1Hello_ = false;
          // Send a request for the version info
          ESP_LL1(TAG, "Requesting version information");
          makeTxDataPacket_(this->txDataQueuedPacket_, RTYPE_VERSION);
          queueTxPacket_(this->txDataQueuedPacket_);
          // Send a request for the keypad info
          ESP_LL1(TAG, "Requesting connected keypads");
          makeTxDataPacket_(this->txDataQueuedPacket_, RTYPE_CONN_KEYPADS);
          queueTxPacket_(this->txDataQueuedPacket_);
        }

        // Allow reception of the next packet
        this->packetStateFlags_ &= ~PSF_RX_FLAGS;
      }
      // ACK Case
      if (this->packetStateFlags_ & PSF_RX_ACK) {
        PanelPacketHeader *pph = (PanelPacketHeader *) this->txDataDequeuedPacket_;
        if ((this->txRetries_ > 0) && (this->txRetries_ < PANEL_MAX_RETRIES)) {
          this->ec_local_.tx_soft_errors++;
        }
        if (this->rxAckPacketSequenceNumber_ != pph->seq_num) {
          ESP_LOGW(TAG, "Received bad sequence number on ACK packet: is: %d, s/b: %d", this->rxAckPacketSequenceNumber_,
                   pph->seq_num);
        } else {
          ESP_LL1(TAG, "TX packet sequence number %d successfully Ack'ed", pph->seq_num);
        }

        // Allow reception and transmission.
        this->packetStateFlags_ &= ~(PSF_RX_FLAGS | PSF_TX_BUSY | PSF_INIT);
      } else if (this->packetStateFlags_ & PSF_TX_BUSY) {
        // If NAK
        if (this->packetStateFlags_ & PSF_RX_NAK) {
          if (this->txRetries_ < PANEL_MAX_RETRIES) {
            this->txRetries_++;
            // Log the type of error
            if (this->packetStateFlags_ & PSF_RX_NAK) {
              ESP_LOGW(TAG, "TX NAK'ed, packet %d, at retry number: %d", this->txDataDequeuedPacket_[1],
                       this->txRetries_);
            }
            // Retransmit the current packet
            this->packetState_ = PRX_TX;
            this->packetStateFlags_ &= ~(PSF_RX_FLAGS);  // Keep TX busy
          } else {
            // The link is really messed up, or there is a bug.  We have to discard the packet
            ESP_LOGE(TAG, "Transmit NAK hard error");
            this->commProblem_ = true;
            this->ec_local_.tx_hard_errors++;
            this->packetStateFlags_ &= ~(PSF_RX_FLAGS | PSF_TX_BUSY);
            // If initial HELLO then back off and wait, then try again
            if (this->packetStateFlags_ & PSF_INIT) {
              ESP_LL1(TAG, "Backing off after comm. failure");
              this->helloBackoffTimer_ = millis();
              this->packetState_ = PRX_HELLO_BACKOFF;
            }
          }
        }
        // If packet transmit timeout
        else if (now - this->txTimer_ > PACKET_TX_TIMEOUT_MS) {
          if (this->txRetries_ < PANEL_MAX_RETRIES) {
            this->txRetries_++;
            // Log the type of error
            ESP_LOGW(TAG, "TX timeout, packet %d, at retry number: %d, _now: %d, _txtimer: %d",
                     this->txDataDequeuedPacket_[1], this->txRetries_, now, this->txTimer_);
            // Retransmit the current packet
            this->packetState_ = PRX_TX;
            this->packetStateFlags_ &= ~(PSF_RX_FLAGS);  // Keep TX busy
          } else {
            // The link is really messed up, or there is a bug.  We have to discard the packet
            ESP_LOGE(TAG, "Transmit time out hard error");
            this->commProblem_ = true;
            this->ec_local_.tx_hard_errors++;
            this->packetStateFlags_ &= ~(PSF_RX_FLAGS | PSF_TX_BUSY);
            // If initial HELLO then back off and wait, then try again
            if (this->packetStateFlags_ & PSF_INIT) {
              ESP_LL1(TAG, "Backing off after comm. failure");
              this->helloBackoffTimer_ = millis();
              this->packetState_ = PRX_HELLO_BACKOFF;
            }
          }
        }
      }
      // If any bad packet
      else if (this->packetStateFlags_ & PSF_BAD_PACKET) {
        ESP_LL1(TAG, "Received bad packet, sending NAK");
        // logDebugHex_("Bad bytes", this->rxDataPacket_, 16);
        //  Packet failed validation send NAK
        makeTxAckNakPacket_(PT_NAK, 0);
        // Transmit it
        txFrame_(&this->txAckNakPacket_);
        // Allow reception of the next packet
        this->ec_local_.rx_bad_packets++;
        this->packetStateFlags_ &= ~(PSF_RX_FLAGS | PSF_TX_BUSY);
      }
      // Ignore NAK outside of TX busy
      else if (this->packetStateFlags_ == PSF_RX_NAK) {
        this->packetStateFlags_ &= ~(PSF_RX_FLAGS | PSF_TX_BUSY);
      }

      // Dequeue next packet if there is one and we are not busy
      if ((this->helloReceived_) && ((this->packetStateFlags_ & PSF_TX_BUSY) == 0) &&
          (deQueueTxPacket_(this->txDataDequeuedPacket_))) {
        this->packetStateFlags_ |= PSF_TX_BUSY;
        this->txRetries_ = 0;
        this->packetState_ = PRX_TX;
      }
      break;
    }  // End case PRX_STATE_IDLE

    case PRX_TX:  // Transmit a packet in the pool
      this->txTimer_ = millis();
      ESP_LL1(TAG, "Transmitting packet number: %d, _txTimer %d", this->txDataDequeuedPacket_[1], this->txTimer_);
      txFrame_(this->txDataDequeuedPacket_);
      this->packetState_ = PRX_STATE_IDLE;
      break;

    case PRX_HELLO_BACKOFF:
      // Wait backoff time for hello and try again
      if (TEST_TIMER(this->helloBackoffTimer_, HELLO_BACKOFF_TIME_MS)) {
        this->packetStateFlags_ = 0;
        this->packetState_ = PRX_STATE_INIT;
      }
      break;

    default:
      // Catch all
      this->packetState_ = PRX_STATE_IDLE;
      break;
  }  // End switch
}

/*
 * Turn on display backlight on all keypads
 */

void Kpa1::backlightOn_() {
  this->backLightTimer_ = millis();
  if (this->queuedKdu_.back_light) {
    return;
  }
  this->queuedKdu_.back_light = true;
  kduEnqueue_(&this->queuedKdu_);
}

/*
 * Monitor the backlight timer and turn off the backlight if
 * the timer expires
 */

void Kpa1::backlightHandler_() {
  if (this->queuedKdu_.back_light) {
    if (TEST_TIMER(this->backLightTimer_, BACKLIGHT_TIME_MS)) {
      this->queuedKdu_.back_light = false;
      // Save a copy of the chime type
      uint8_t chime_save = this->queuedKdu_.chime;
      // If the alarm is in the disarm state, temporarly override the chime type
      this->queuedKdu_.chime = CHIME_NONE;
      // Send the backlight off message to the display
      kduEnqueue_(&this->queuedKdu_);
      
      // Restore the chime type
      this->queuedKdu_.chime = chime_save;
      kduEnqueue_(&this->queuedKdu_);
    }
  }
}

/*
 * Keypad Update handler - If a message is ready and the update timer has expired it will queue a TX message to the kpa1
 * to update the keypad LCD's, buzzer, and status LED's
 */

void Kpa1::keypadUpdateHandler_() {
  if (TEST_TIMER(this->keypadUpdateTimer_, KEYPAD_UPDATE_TIME)) {
    if (kduDequeue_(&this->queuedKdu_)) {
      this->keypadUpdateTimer_ = millis();
      // Make TX data packet from Keypad Display Packet, and add it to the TX queue
      makeTxDataPacket_(this->txDataQueuedPacket_, RTYPE_UPDATE_KEYPAD, &this->queuedKdu_);
      queueTxPacket_(this->txDataQueuedPacket_);
    }
  }
}

/*
 * Periodically request the error counters from the kpa1
 */

void Kpa1::remoteErrorCountersHandler_() {
  if (TEST_TIMER(this->remoteErrorCounterTimer_, REMOTE_ERROR_COUNTER_UPDATE_TIME_MS)) {
    this->remoteErrorCounterTimer_ = millis();
    ESP_LL1(TAG, "Requesting remote error counter update");
    makeTxDataPacket_(this->txDataQueuedPacket_, RTYPE_SEND_ERROR_COUNTERS);
    queueTxPacket_(this->txDataQueuedPacket_);
  }
}

/*
 * Ready led handler. Throttles the update of the ready LED.
 * Handles chime when sensor opened.
 */

void Kpa1::readyLedHandler_() {
  uint8_t chime_new = CHIME_NONE, chime_save;
  
  // Don't update anything if ARMING, PENDING, or TRIGGERED
  if(this->alarmState_ == esphome::alarm_control_panel::ACP_STATE_ARMING ||
    this->alarmState_ == esphome::alarm_control_panel::ACP_STATE_PENDING ||
    this->alarmState_ == esphome::alarm_control_panel::ACP_STATE_TRIGGERED) {
    return;
  }
     
  
  if (TEST_TIMER(this->readyLedTimer_, READY_LED_UPDATE_TIME_MS)) {
    if (this->fastReadyLed_ != this->queuedKdu_.ready) {
      this->readyLedTimer_ = millis();
      // If the alarm state is disarmed,
      // Update the display text as well
      if (this->alarmState_ == esphome::alarm_control_panel::ACP_STATE_DISARMED) {
        lcdCopyString_(0, 0, DISARMED_TEXT);
        if (this->fastReadyLed_) {
          lcdCopyString_(1, 0, READY_TO_ARM_TEXT);
        } else {
          chime_new = (this->fastChime_) ? CHIME_THREE_TIMES : CHIME_NONE;  // Sensor opened
          lcdCopyString_(1, 0, NOT_READY_TEXT);
        }
      }
      chime_save = this->queuedKdu_.chime;
      this->queuedKdu_.chime = chime_new;
      this->queuedKdu_.ready = this->fastReadyLed_;
      kduEnqueue_(&this->queuedKdu_);
      this->queuedKdu_.chime = chime_save;
      kduEnqueue_(&this->queuedKdu_);
      
    }
  }
}

/*
 * Code digit receiver has received a code and command.
 * Process it here.
 */

void Kpa1::receiveCodeDigitsHandler_() {
  // If the panel didn't have a state change for a command within a certain time window, reset the code
  // and command receive flag as the command or code was likely invalid.
  if ((this->codeAndCommandReceived_ == true) && TEST_TIMER(this->validCommandTimer_, COMMAND_VALID_FLAG_TIME_MS)) {
    this->codeAndCommandReceived_ = false;
  }

  switch (this->codeReceiverState_) {
    case CR_IDLE:
      if (this->keypadDigitCount_) {  // Something to append?
        backlightOn_();               // Turn on backlight
        this->codeDigitSourceAddress_ = this->keypadDigitSourceAddress_;
        this->codeDigitCount_ = 0;
        this->codeReceiverTimer_ = millis();
        this->codeReceiverState_ = CR_BUSY;
      }
      break;

    case CR_BUSY:
      // Interdigit time out check
      if (TEST_TIMER(this->codeReceiverTimer_, CODE_INTERDIGIT_TIMEOUT_MS)) {
        ESP_LL1(TAG, "Interdigit timeout reached");
        this->codeDigitCount_ = 0;
        this->codeDigitSourceAddress_ = 0;
        this->keypadDigitCount_ = 0;
        this->codeReceiverState_ = CR_IDLE;
      }
      // Check for digits received
      if (this->keypadDigitCount_) {
        // Only accept digits from the address which initially sent them
        if (this->codeDigitSourceAddress_ == this->keypadDigitSourceAddress_) {
          // Determine if we can fit what was sent
          uint8_t transfer_count = this->keypadDigitCount_;
          if (this->codeDigitCount_ + transfer_count > (CODE_DIGITS_MAX + 1)) {
            ESP_LL1(TAG, "Received extra digits, truncating");
            transfer_count = (CODE_DIGITS_MAX + 1) - this->codeDigitCount_;
          }
          // Transfer new digits to code in process buffer
          for (int i = 0; i < transfer_count; i++) {
            // Check for start over key
            if (keypadDigitsReceived_[i] == CODE_START_OVER_KEY) {
              // Reset code digit receiver
              this->keypadDigitCount_ = 0;
              this->codeReceiverState_ = CR_IDLE;
              ESP_LL1(TAG, "User pressed code start over key, resetting code receiver");
              return;
            }
            // Handle assistance key
            if ((keypadDigitsReceived_[i] == 'A' ) ||( keypadDigitsReceived_[i] == 'B') || 
                (keypadDigitsReceived_[i] == 'C') || (keypadDigitsReceived_[i] == 'D')) {
              uint8_t index = keypadDigitsReceived_[i] - 'A';
              this->assistanceKey_[index] = true;
              // Reset code digit receiver
              this->keypadDigitCount_ = 0;
              this->codeReceiverState_ = CR_IDLE;
              ESP_LL1(TAG, "User pressed an assistance key, resetting code receiver");
              return;
            }
            
            // Add key to code digit buffer
            this->codeInProcess_[this->codeDigitCount_ + i] = keypadDigitsReceived_[i];
          }
          this->codeDigitCount_ += transfer_count;
          this->keypadDigitCount_ = 0;  // Release keypad buffer for next batch of keys.

          // Code and command ready to be sent to the alarm controller?
          if (this->codeDigitCount_ == (CODE_DIGITS_MAX) + 1) {
            char command = (char) this->codeInProcess_[CODE_DIGITS_MAX];
            this->codeInProcess_[CODE_DIGITS_MAX] = 0;
            std::string keypad_code = (char *) this->codeInProcess_;
            bool valid_command = false;
            switch (command) {
              case '1':  // OFF
                valid_command = true;
                break;

              case '2':  // AWAY
                valid_command = true;
                break;

              case '3':  // STAY
                valid_command = true;
                break;

              default:
                break;
            }

            if (valid_command) {
              ESP_LOGI(TAG, "Code and command received from keypad address %d", this->codeDigitSourceAddress_);
              // Signal code and command have been received. Panel needs to validate
              this->codeAndCommandReceived_ = true;
              this->validCommandTimer_ = millis();

              // Send the command to the alarm control panel
              switch (command) {
                case '1':  // OFF
                  id(this->acp_).disarm(keypad_code);
                  break;

                case '2':  // AWAY
                  id(this->acp_).arm_away(keypad_code);
                  break;

                case '3':  // STAY
                  id(this->acp_).arm_home(keypad_code);
                  break;
              }
            }

            this->codeDigitCount_ = 0;
            this->codeDigitSourceAddress_ = 0;
            this->codeReceiverState_ = CR_IDLE;
          }
          // Reset interdigit timer
          this->codeReceiverTimer_ = millis();
        } else {
          // Discard data from a different keypad
          ESP_LL1(TAG, "Received code digits from keypad address %02X, ignoring...", this->keypadDigitSourceAddress_);
          this->keypadDigitCount_ = 0;
        }
      }

      break;

    default:
      this->codeReceiverState_ = CR_IDLE;
      break;
  }
}

/*
 * Return value of a specific assistance key
 */

bool Kpa1::getAssistanceKey_(uint8_t key) {
  if(key >= MAX_ASSISTANCE_KEY) {
    return false;
  }
  if( this->assistanceKey_[key] == false) {
    return false;
  }
  bool ak = this->assistanceKey_[key];
  this->assistanceKey_[key] = false;
  return ak;
}

/*
 * Setup function
 */

void Kpa1::setup() {
  this->stuffedRxState_ = SRX_STATE_IDLE;
  this->rxFrameState_ = RF_STATE_IDLE;
  this->packetState_ = PRX_STATE_INIT;
  this->packetStateFlags_ = PSF_CLEAR;
  this->lastRxSeqNum_ = this->txSeqNum_ = 0;
  this->txDataPoolHead_ = this->txDataPoolTail_ = 0;
  this->kduHead_ = this->kduTail_ = 0;
  this->txRetries_ = 0;
  this->keypadCommand_ = 0;
  this->keypadDigitCount_ = 0;
  this->keypadDigitSourceAddress_ = 0;
  this->codeDigitCount_ = 0;
  this->codeReceiverState_ = CR_IDLE;
  this->codeDigitSourceAddress_ = 0;
  this->keypadBacklightState_ = false;
  this->kpa1Hello_ = false;
  this->helloReceived_ = false;
  this->codeAndCommandReceived_ = false;
  this->commProblem_ = false;
  this->fastReadyLed_ = false;
  this->fastChime_ = false;
 

  uint32_t now = millis();
  this->powerOnTimer_ = now;
  this->backLightTimer_ = now;
  this->keypadUpdateTimer_ = now;
  this->codeReceiverTimer_ = now;
  this->readyLedTimer_ = now;
  this->txTimer_ = now;
  this->remoteErrorCounterTimer_ = now;
  this->validCommandTimer_ = now;
  this->helloBackoffTimer_ = now;

  // Initialize KDU
  this->queuedKdu_.armed = false;
  this->queuedKdu_.ready = false;
  this->queuedKdu_.back_light = false;
  this->queuedKdu_.keypad_address = 0xFF;  // All keypads
  this->queuedKdu_.chime = CHIME_NONE;
  this->queuedKdu_.len_line1 = MAX_KEYPAD_LINE;
  this->queuedKdu_.len_line2 = MAX_KEYPAD_LINE;
  memset(this->queuedKdu_.line1, ' ', MAX_KEYPAD_LINE);
  memset(this->queuedKdu_.line2, ' ', MAX_KEYPAD_LINE);

  // Clear local error counters
  memset(&this->ec_local_, 0, sizeof(ErrorCountersLocal));
  // Clear remote error counters
  memset(&this->ec_remote_, 0, sizeof(ErrorCountersRemote));
  // Clear the keypad info
  memset(&this->ki_, 0, sizeof(PanelKeypadInfo));
  // Clear assistance keys
  memset(this->assistanceKey_, false, MAX_ASSISTANCE_KEY);
}

/*
 * loop function
 */

void Kpa1::loop() {
  // Service the state machines
  rxFrame_();
  commStateMachineHandler_();
  keypadUpdateHandler_();
  receiveCodeDigitsHandler_();
  backlightHandler_();
  readyLedHandler_();
  remoteErrorCountersHandler_();
}

/*
 * dump config function
 */

void Kpa1::dump_config() {
  ESP_LOGCONFIG(TAG, "KPA1:");
  ESP_LOGCONFIG(TAG, "Kpa1 connected: %s", (this->helloReceived_) ? "true" : "false");
}

void Kpa1::dump_error_counters() {
  ESP_LOGI(TAG, "************ Local Error Counters *************");
  ESP_LOGI(TAG, "TX Soft Errors                 : %d", ec_local_.tx_soft_errors);
  ESP_LOGI(TAG, "TX Hard Errors                 : %d", ec_local_.tx_hard_errors);
  ESP_LOGI(TAG, "TX Buffer Pool Overflow Errors : %d", ec_local_.tx_buffer_pool_overflow_errors);
  ESP_LOGI(TAG, "RX Bad Packets                 : %d", ec_local_.rx_bad_packets);
  ESP_LOGI(TAG, "RX Frame Timeouts              : %d", ec_local_.rx_frame_timeouts);
  ESP_LOGI(TAG, "************ Remote Error Counters *************");
  ESP_LOGI(TAG, "TX Soft Errors                 : %d", ec_remote_.tx_soft_errors);
  ESP_LOGI(TAG, "TX Hard Errors                 : %d", ec_remote_.tx_hard_errors);
  ESP_LOGI(TAG, "TX Buffer Pool Overflow Errors : %d", ec_remote_.tx_buffer_pool_overflow_errors);
  ESP_LOGI(TAG, "RX Bad Packets                 : %d", ec_remote_.rx_bad_packets);
  ESP_LOGI(TAG, "RX Frame Timeouts              : %d", ec_remote_.rx_frame_timeouts);
  ESP_LOGI(TAG, "ECP Parity Errors              : %d", ec_remote_.ecp_parity_errors);
  ESP_LOGI(TAG, "ECP Checksum Errors            : %d", ec_remote_.ecp_checksum_errors);
}

/*
 * Set alarm control panel object
 */

void Kpa1::set_acp(alarm_control_panel::AlarmControlPanel *acp_id) { this->acp_ = acp_id; }

/*
 * Control keypad entry silent mode
 */

void Kpa1::set_keypad_entry_silent(bool silent) { this->keypadEntrySilent_ = silent; }

/*
 * Control keypad exit silent mode
 */

void Kpa1::set_keypad_exit_silent(bool silent) { this->keypadExitSilent_ = silent; }

/*
 * Control keypad alarm silent mode
 */

void Kpa1::set_keypad_alarm_silent(bool silent) { this->keypadAlarmSilent_ = silent; }

/*
 * Update the keypad ready LED. Called from YAML when ready state changes.
 */

void Kpa1::update_system_ready(bool ready) {
  this->fastReadyLed_ = ready;  // Save a copy for testing later
}

/*
 * Update the entry chime state
 */

void Kpa1::update_system_entry_chime(bool chime) {
  this->fastChime_ = chime;  // Save copy for testing later
}

/*
 * Return whether or not there are any faulted zones
 */

bool Kpa1::check_for_zone_faults(uint64_t zone_state_bits, uint64_t zone_bits_mask) {
  return (bool) zone_state_bits & zone_bits_mask;
}

/*
 * Return true if we can't communicate with the kpa1
 */

bool Kpa1::get_keypad_comm_problem() { return this->commProblem_; }


/*
 * Return the value of the assistance key A
 */


bool Kpa1::get_assistance_key_a() {
  return getAssistanceKey_(0);
}

/*
 * Return the value of the assistance key B
 */

bool Kpa1::get_assistance_key_b() {
  return getAssistanceKey_(1);
}

/*
 * Return the value of the assistance key C
 */

bool Kpa1::get_assistance_key_c() {
  return getAssistanceKey_(2);
}

/*
 * Return the value of the assistance key D
 */
bool Kpa1::get_assistance_key_d() {
  return getAssistanceKey_(3);
}

/*
 * Return list of kpa1 addresses
 */

uint8_t Kpa1::get_keypad_count() {
  uint8_t i, count = 0;
  for (i = 0; i < KP_INFO_MAX_KEYPADS; i++) {
    if (ki_.info[i].valid) {
      count++;
    }
  }
  ESP_LOGI(TAG, "Number of keypads: %d", count);
  return count;
}

/*
 * Return keypad model for a given address
 */

const char *Kpa1::get_keypad_info() {
  const char *model;
  char elem[4 + KP_MODEL_LEN + 1];
  static char info[KP_INFO_MAX_KEYPADS * (KP_MODEL_LEN + 4) + 2];

  info[0] = 0;

  for (int i = 0; i < KP_INFO_MAX_KEYPADS; i++) {
    if (ki_.info[i].valid) {
      // Set model string
      if ((ki_.info[i].model[3] == 0x04) && (ki_.info[i].model[4] == 0x04) && (ki_.info[i].model[5] == 0x04)) {
        model = "6160";
      } else if ((ki_.info[i].model[3] == 0x04) && (ki_.info[i].model[4] == 0x06) && (ki_.info[i].model[5] == 0x04)) {
        model = "6139";
      } else {
        model = "UNK";
      }
      snprintf(elem, sizeof(elem), "%s@%02d,", model, 16 + i);
      elem[sizeof(elem) - 1] = 0;
      // ESP_LL1(TAG, "Keypad elem: %s", elem);
      strcat(info, elem);
    }
  }
  // Delete extra comma
  if (strlen(info)) {
    info[strlen(info) - 1] = 0;
  }
  // Return info string
  ESP_LOGI(TAG, "Keypad info: %s", info);
  return (const char *) info;
}

/*
 * This is called from YAML when the alarm panel state changes
 */

void Kpa1::update_alarm_state(uint8_t status) {
  // Save a copy of the alarm state
  this->alarmState_ = status;

  switch (status) {
    case esphome::alarm_control_panel::ACP_STATE_DISARMED:
      backlightOn_();
      lcdCopyString_(0, 0, DISARMED_TEXT);
      if (this->queuedKdu_.ready) {
        lcdCopyString_(1, 0, READY_TO_ARM_TEXT);
      } else {
        lcdCopyString_(1, 0, NOT_READY_TEXT);
      }
      this->queuedKdu_.chime = (this->codeAndCommandReceived_) ? CHIME_ONCE : CHIME_NONE;
      this->queuedKdu_.armed = false;
      break;

    case esphome::alarm_control_panel::ACP_STATE_ARMED_HOME:
      lcdCopyString_(0, 0, ARMED_HOME_TEXT);
      lcdCopyString_(1, 0, "");  // Blank line
      this->queuedKdu_.chime = CHIME_NONE;
      this->queuedKdu_.armed = true;
      break;

    case esphome::alarm_control_panel::ACP_STATE_ARMED_AWAY:
      lcdCopyString_(0, 0, ARMED_AWAY_TEXT);
      lcdCopyString_(1, 0, "");  // Blank line
      this->queuedKdu_.chime = CHIME_NONE;
      this->queuedKdu_.armed = true;
      break;

    case esphome::alarm_control_panel::ACP_STATE_ARMED_NIGHT:
      lcdCopyString_(0, 0, ARMED_NIGHT_TEXT);
      lcdCopyString_(1, 0, "");  // Blank line
      this->queuedKdu_.chime = (this->codeAndCommandReceived_) ? CHIME_ONCE : CHIME_NONE;
      this->queuedKdu_.armed = true;
      break;

    case esphome::alarm_control_panel::ACP_STATE_ARMED_VACATION:
      lcdCopyString_(0, 0, ARMED_VACATION_TEXT);
      lcdCopyString_(1, 0, "");  // Blank line
      this->queuedKdu_.chime = (this->codeAndCommandReceived_) ? CHIME_ONCE : CHIME_NONE;
      this->queuedKdu_.armed = true;
      break;

    case esphome::alarm_control_panel::ACP_STATE_ARMED_CUSTOM_BYPASS:
      lcdCopyString_(0, 0, ARMED_CUSTOM_BYPASS_TEXT);
      lcdCopyString_(1, 0, "");  // Blank line
      this->queuedKdu_.chime = (this->codeAndCommandReceived_) ? CHIME_ONCE : CHIME_NONE;
      this->queuedKdu_.armed = true;
      break;

    case esphome::alarm_control_panel::ACP_STATE_PENDING:

      backlightOn_();

      if (this->keypadEntrySilent_) {
        this->queuedKdu_.chime = CHIME_NONE;
      } else {
        this->queuedKdu_.chime = CHIME_FAST_REPEATING;
      }
      lcdCopyString_(0, 0, ALARM_PENDING_TEXT);
      lcdCopyString_(1, 0, ENTER_CODE_NOW_TEXT);
      break;

    case esphome::alarm_control_panel::ACP_STATE_ARMING:
      if (this->codeAndCommandReceived_) {
        this->queuedKdu_.chime = CHIME_ONCE;  // Acknowledge arming command
        kduEnqueue_(&this->queuedKdu_);
        this->queuedKdu_.chime = CHIME_NONE;
        
      }
      if (this->keypadExitSilent_) {
        this->queuedKdu_.chime = CHIME_NONE;
      } else {
        this->queuedKdu_.chime = CHIME_SLOW_REPEATING;
      }
      lcdCopyString_(0, 0, ALARM_ARMING_TEXT);
      lcdCopyString_(1, 0, LEAVE_NOW_TEXT);
      break;

    case esphome::alarm_control_panel::ACP_STATE_DISARMING:
      this->queuedKdu_.chime = CHIME_NONE;
      lcdCopyString_(0, 0, DISARMING_TEXT);
      lcdCopyString_(1, 0, "");  // Blank line
      break;

    case esphome::alarm_control_panel::ACP_STATE_TRIGGERED:
      if (this->keypadAlarmSilent_) {
        this->queuedKdu_.chime = CHIME_NONE;
      } else {
        this->queuedKdu_.chime = CHIME_LOUD;
      }
      lcdCopyString_(0, 0, TRIGGERED_TEXT);
      lcdCopyString_(1, 0, "");  // Blank line
      break;
  }
  // Queue to send to keypads
  kduEnqueue_(&this->queuedKdu_);
  // Reset chime once
  if (this->queuedKdu_.chime == CHIME_ONCE) {
    this->queuedKdu_.chime = CHIME_NONE;
    kduEnqueue_(&this->queuedKdu_);
  }
  // Reset code and command received flag
  this->codeAndCommandReceived_ = false;
}

}  // namespace kpa1
}  // namespace esphome
