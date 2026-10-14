#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>

#define SERVER_PORT 6000
#define MAX_FRAME_LEN 64

static void print_hex(const uint8_t *buf, size_t len) {
  for (size_t i = 0; i < len; ++i) {
    printf("%02X ", buf[i]);
  }
  printf("\n");
}

int main(void) {
  int server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
  if (server_fd < 0) {
    perror("socket");
    return 1;
  }

  int opt = 1;
  if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
    perror("setsockopt");
    close(server_fd);
    return 1;
  }

  struct sockaddr_in addr;
  memset(&addr, 0, sizeof(addr));
  addr.sin_family = AF_INET;
  addr.sin_addr.s_addr = htonl(INADDR_ANY);
  addr.sin_port = htons(SERVER_PORT);

  if (bind(server_fd, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("bind");
    close(server_fd);
    return 1;
  }

  if (listen(server_fd, 5) < 0) {
    perror("listen");
    close(server_fd);
    return 1;
  }

  printf("TCP receive server listening on port %d\n", SERVER_PORT);

  while (1) {
    struct sockaddr_in client_addr;
    socklen_t client_len = sizeof(client_addr);
    int client_fd =
        accept(server_fd, (struct sockaddr *)&client_addr, &client_len);
    if (client_fd < 0) {
      perror("accept");
      continue;
    }

    char ipbuf[INET_ADDRSTRLEN] = {0};
    inet_ntop(AF_INET, &client_addr.sin_addr, ipbuf, sizeof(ipbuf));
    printf("client connected: %s:%d\n", ipbuf, ntohs(client_addr.sin_port));

    while (1) {
      uint8_t frame[MAX_FRAME_LEN] = {0};
      ssize_t n = recv(client_fd, frame, sizeof(frame), 0);
      if (n <= 0) {
        printf("client disconnected\n");
        break;
      }

      printf("received %zd bytes\n", n);
      print_hex(frame, (size_t)n);

      printf("raw bytes: ");
      for (ssize_t i = 0; i < n; ++i) {
        printf("%02X ", frame[i]);
      }
      printf("\n");
    }

    close(client_fd);
  }

  close(server_fd);
  return 0;
}
