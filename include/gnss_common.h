
/**
 * Shared GNSS pipeline types and helpers currently used by reactors.
 */

#ifndef GNSS_COMMON_H
#define GNSS_COMMON_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

// Ring buffer capacity for DataForwarder.
#define MEM_BUFFER_SIZE 1000

// Packet and protocol sizing constants.
#define MAX_PACKET_SIZE 2048
#define MAX_RTCM_MESSAGES_PER_PACKET 10
#define E2E_HEADER_SIZE 16
#define E2E_PREAMBLE_0 0x0F
#define E2E_PREAMBLE_1 0xF0
#define E2E_PREAMBLE_2 0x5A
#define E2E_PREAMBLE_3 0x03
#define RTCM3_PREAMBLE 0xD3
#define RTCM3_HEADER_SIZE 3
#define RTCM3_CRC_SIZE 3
#define RTCM3_OVERHEAD (RTCM3_HEADER_SIZE + RTCM3_CRC_SIZE)

typedef struct {
  uint16_t message_id;
  uint16_t content_length;
  int offset;
  int total_size;
  bool is_complete;
  bool is_corrupted;
  int bytes_transmitted;
} rtcm3_message_t;

typedef struct {
  uint16_t e2e_length;
  uint16_t e2e_counter;
  uint32_t e2e_data_id;
  int rtcm_count;
  rtcm3_message_t rtcm_messages[MAX_RTCM_MESSAGES_PER_PACKET];
} e2e_packet_info_t;

typedef struct {
  uint8_t data[MAX_PACKET_SIZE];
  int bytes;
  instant_t generation_time;
  e2e_packet_info_t packet_info;
} data_packet_t;

static inline void print_fifo_overflow(instant_t time_ms, int fifo_cap, int fifo_level,
                                       int bytes_arrived, interval_t elapsed_us,
                                       int bytes_lost, int lost_start, int lost_end,
                                       int cumulative_loss, int event_num) {
  printf("\n[FIFO OVERFLOW] t=%lld ms\n", time_ms);
  printf("  fifo_capacity=%d bytes, fifo_level=%d bytes\n", fifo_cap, fifo_level);
  printf("  bytes_arrived=%d (elapsed=%lld us)\n", bytes_arrived, elapsed_us);
  printf("  bytes_lost=%d (range=%d-%d)\n", bytes_lost, lost_start, lost_end);
  printf("  cumulative_loss=%d, event=%d\n\n", cumulative_loss, event_num);
}

static inline void print_buffer_overflow(instant_t time_ms, int need, int available,
                                         int buffer_size, int event_num, int total_dropped) {
  printf("\n[BUFFER OVERFLOW] t=%lld ms\n", time_ms);
  printf("  need=%d bytes, available=%d bytes, shortage=%d bytes\n",
    need, available, need - available);
  printf("  buffer_size=%d bytes, recommended_size=2048 bytes\n", buffer_size);
  printf("  event=%d, total_dropped=%d bytes\n\n", event_num, total_dropped);
}

static inline bool validate_e2e_preamble(const uint8_t* data) {
  return (data[0] == E2E_PREAMBLE_0 &&
          data[1] == E2E_PREAMBLE_1 &&
          data[2] == E2E_PREAMBLE_2 &&
          data[3] == E2E_PREAMBLE_3);
}

static inline uint16_t extract_rtcm3_length(const uint8_t* header) {
  return ((uint16_t)(header[1] & 0x03) << 8) | header[2];
}

static inline uint16_t extract_rtcm3_message_id(const uint8_t* content) {
  return ((uint16_t)content[0] << 4) | (content[1] >> 4);
}

static inline int parse_e2e_packet(data_packet_t* packet) {
  if (packet->bytes < E2E_HEADER_SIZE) {
    return 0;
  }

  if (!validate_e2e_preamble(packet->data)) {
    printf("[ERROR] Invalid E2E preamble\n");
    return 0;
  }

  packet->packet_info.e2e_length = ((uint16_t)packet->data[4] << 8) | packet->data[5];
  packet->packet_info.e2e_counter = ((uint16_t)packet->data[6] << 8) | packet->data[7];
  packet->packet_info.e2e_data_id = ((uint32_t)packet->data[8] << 24) |
                                     ((uint32_t)packet->data[9] << 16) |
                                     ((uint32_t)packet->data[10] << 8) |
                                     packet->data[11];

  int rtcm_count = 0;
  int offset = E2E_HEADER_SIZE;

  while (offset < packet->bytes && rtcm_count < MAX_RTCM_MESSAGES_PER_PACKET) {
    if (packet->data[offset] != RTCM3_PREAMBLE) {
      printf("[WARNING] Expected RTCM3 preamble at offset %d, got 0x%02X\n",
             offset, packet->data[offset]);
      break;
    }

    uint16_t content_length = extract_rtcm3_length(&packet->data[offset]);
    int total_size = RTCM3_OVERHEAD + content_length;

    if (offset + total_size > packet->bytes) {
      printf("[WARNING] RTCM3 message at offset %d exceeds packet boundary\n", offset);
      break;
    }

    uint16_t message_id = 0;
    if (content_length >= 2) {
      message_id = extract_rtcm3_message_id(&packet->data[offset + RTCM3_HEADER_SIZE]);
    }

    packet->packet_info.rtcm_messages[rtcm_count].message_id = message_id;
    packet->packet_info.rtcm_messages[rtcm_count].content_length = content_length;
    packet->packet_info.rtcm_messages[rtcm_count].offset = offset;
    packet->packet_info.rtcm_messages[rtcm_count].total_size = total_size;
    packet->packet_info.rtcm_messages[rtcm_count].is_complete = true;
    packet->packet_info.rtcm_messages[rtcm_count].is_corrupted = false;
    packet->packet_info.rtcm_messages[rtcm_count].bytes_transmitted = 0;

    rtcm_count++;
    offset += total_size;
  }

  packet->packet_info.rtcm_count = rtcm_count;
  return rtcm_count;
}

#endif // GNSS_COMMON_H