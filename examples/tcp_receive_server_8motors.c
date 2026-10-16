#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "example_soem/motor_layer.h"
#include "example_soem/net_compat.h"

#define SERVER_PORT 6000
#define MAX_PACKET_SIZE 256
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

static void print_raw_frame_hex(const RawMotorFrame *frame) {
  printf("CAN id=%u flags=0x%02X len=%u data=", (unsigned)frame->can_id,
         (unsigned)frame->flags, (unsigned)frame->len);
  for (uint8_t i = 0; i < frame->len; ++i) {
    printf("%02X ", frame->data[i]);
  }
  printf("\n");
}

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
  encos_socket_t server_fd;

  if (!encos_net_startup()) {
    encos_socket_error("network startup");
    return 1;
  }

  server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (server_fd == ENCOS_INVALID_SOCKET) {
    encos_socket_error("socket");
    print_status(false, "TCP socket create failed");
    encos_net_cleanup();
    return 1;
  }

  {
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, (const char *)&opt,
                   sizeof(opt)) < 0) {
      encos_socket_error("setsockopt");
      encos_close_socket(server_fd);
      encos_net_cleanup();
      print_status(false, "TCP socket set SO_REUSEADDR failed");
      return 1;
    }
  }

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  if (inet_pton(AF_INET, bind_ip, &addr.sin_addr) != 1) {
    fprintf(stderr, "Invalid IP address: %s\n", bind_ip);
    encos_close_socket(server_fd);
    encos_net_cleanup();
    print_status(false, "Invalid TCP bind IP");
    return 1;
  }
  addr.sin_port = htons(SERVER_PORT);

  if (bind(server_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    encos_socket_error("bind");
    encos_close_socket(server_fd);
    encos_net_cleanup();
    print_status(false, "TCP bind failed");
    return 1;
  }

  if (listen(server_fd, 8) < 0) {
    encos_socket_error("listen");
    encos_close_socket(server_fd);
    encos_net_cleanup();
    print_status(false, "TCP listen failed");
    return 1;
  }

  print_status(true, "TCP receiver ready");
  printf("TCP 8-motor receive server listening on %s:%d\n", bind_ip,
         SERVER_PORT);

  while (1) {
    struct sockaddr_in client_addr;
    encos_socklen_t client_len = sizeof(client_addr);
    encos_socket_t client_fd =
        accept(server_fd, (struct sockaddr *)&client_addr, &client_len);
    if (client_fd == ENCOS_INVALID_SOCKET) {
      encos_socket_error("accept");
      continue;
    }

    char ipbuf[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, &client_addr.sin_addr, ipbuf, sizeof(ipbuf));
    printf("\033[32m[OK]\033[0m TCP client connected: %s:%d\n", ipbuf,
           ntohs(client_addr.sin_port));

    while (1) {
      uint8_t raw[MAX_PACKET_SIZE] = {0};
      encos_ssize_t n = recv(client_fd, (char *)raw, sizeof(raw), 0);
      if (n <= 0) {
        printf("TCP client disconnected\n");
        break;
      }

      size_t count = 0;
      RawMotorFrame frames[MAX_MOTORS] = {0};
      count = parse_motor_batch(raw, (size_t)n, frames, MAX_MOTORS);

      printf("Received TCP batch (%lld bytes), %zu frame(s)\n", (long long)n,
             count);
      if (count == 0) {
        printf("No complete frame parsed\n");
        continue;
      }

      for (size_t i = 0; i < count; ++i) {
        print_decoded_motor(&frames[i]);
      }
    }

    encos_close_socket(client_fd);
  }

  encos_close_socket(server_fd);
  encos_net_cleanup();
  return 0;
}
