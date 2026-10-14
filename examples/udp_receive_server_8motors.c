#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include "example_soem/motor_layer.h"

#define SERVER_PORT 5000
#define MAX_PACKET_SIZE 512
#define MAX_MOTORS 8

static void print_status(bool ok, const char *msg) {
  if (ok) {
    printf("\033[32m[OK]\033[0m %s\n", msg);
  } else {
    printf("\033[31m[FAIL]\033[0m %s\n", msg);
  }
}

#pragma pack(push, 1)
typedef struct {
  uint32_t can_id;
  uint8_t flags;
  uint8_t len;
  uint8_t data[8];
} RawMotorFrame;
#pragma pack(pop)

static void print_decoded_motor(const RawMotorFrame *frame) {
  printf("motor(id=%u, flags=0x%02X, len=%u): ", (unsigned)frame->can_id,
         (unsigned)frame->flags, (unsigned)frame->len);
  for (uint8_t i = 0; i < frame->len; ++i) {
    printf("%02X ", frame->data[i]);
  }
  printf("\n");
}

static size_t parse_motor_batch(const uint8_t *buf, size_t len,
                                RawMotorFrame *frames, size_t max_frames) {
  size_t out = 0;
  size_t offset = 0;

  while (offset + sizeof(RawMotorFrame) <= len && out < max_frames) {
    RawMotorFrame frame;
    memcpy(&frame, buf + offset, sizeof(frame));
    frames[out++] = frame;
    offset += sizeof(RawMotorFrame);
  }

  return out;
}

int main(int argc, char **argv) {
  const char *bind_ip = (argc > 1) ? argv[1] : "0.0.0.0";

  int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
  if (sock < 0) {
    perror("socket");
    print_status(false, "UDP socket create failed");
    return 1;
  }

  int opt = 1;
  if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    perror("setsockopt");
    close(sock);
    print_status(false, "UDP socket set SO_REUSEADDR failed");
    return 1;
  }

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  if (inet_pton(AF_INET, bind_ip, &addr.sin_addr) != 1) {
    fprintf(stderr, "Invalid IP address: %s\n", bind_ip);
    close(sock);
    print_status(false, "Invalid UDP bind IP");
    return 1;
  }
  addr.sin_port = htons(SERVER_PORT);

  if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("bind");
    close(sock);
    print_status(false, "UDP bind failed");
    return 1;
  }

  print_status(true, "UDP receiver ready");
  printf("UDP 8-motor receive server listening on %s:%d\n", bind_ip,
         SERVER_PORT);

  while (1) {
    uint8_t raw[MAX_PACKET_SIZE] = {0};
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    ssize_t n = recvfrom(sock, raw, sizeof(raw), 0,
                         (struct sockaddr *)&client_addr, &client_len);
    if (n <= 0) {
      continue;
    }

    char ipbuf[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, &client_addr.sin_addr, ipbuf, sizeof(ipbuf));

    RawMotorFrame frames[MAX_MOTORS] = {0};
    size_t count = parse_motor_batch(raw, (size_t)n, frames, MAX_MOTORS);

    printf("\033[32m[OK]\033[0m UDP packet from %s:%d => %zu frame(s), %zd "
           "bytes\n",
           ipbuf, ntohs(client_addr.sin_port), count, (size_t)n);
    for (size_t i = 0; i < count; ++i) {
      print_decoded_motor(&frames[i]);
    }
  }

  close(sock);
  return 0;
}
