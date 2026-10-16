#pragma once

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <winsock2.h>
#include <ws2tcpip.h>

#include <stdio.h>

typedef SOCKET encos_socket_t;
typedef int encos_socklen_t;
typedef int encos_ssize_t;

#define ENCOS_INVALID_SOCKET INVALID_SOCKET

static int encos_net_startup(void) {
  WSADATA data;
  return WSAStartup(MAKEWORD(2, 2), &data) == 0;
}

static void encos_net_cleanup(void) { WSACleanup(); }

static int encos_close_socket(encos_socket_t socket_fd) {
  return closesocket(socket_fd);
}

static int encos_set_receive_timeout(encos_socket_t socket_fd, int timeout_ms) {
  return setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO,
                    (const char *)&timeout_ms, sizeof(timeout_ms));
}

static void encos_socket_error(const char *operation) {
  fprintf(stderr, "%s failed: Winsock error %d\n", operation,
          WSAGetLastError());
}

#else

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <stdio.h>

typedef int encos_socket_t;
typedef socklen_t encos_socklen_t;
typedef ssize_t encos_ssize_t;

#define ENCOS_INVALID_SOCKET (-1)

static int encos_net_startup(void) { return 1; }

static void encos_net_cleanup(void) {}

static int encos_close_socket(encos_socket_t socket_fd) {
  return close(socket_fd);
}

static int encos_set_receive_timeout(encos_socket_t socket_fd, int timeout_ms) {
  struct timeval timeout;
  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms % 1000) * 1000;
  return setsockopt(socket_fd, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                    sizeof(timeout));
}

static void encos_socket_error(const char *operation) { perror(operation); }

#endif
