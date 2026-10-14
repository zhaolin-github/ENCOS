#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <poll.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

extern "C" {
#include "example_soem/motor_layer.h"
}

namespace {

constexpr int kMotorCount = 8;
constexpr float kPi = 3.14159265358979323846F;
constexpr float kPositionMinRad = -12.5F;
constexpr float kPositionMaxRad = 12.5F;
constexpr float kSpeedMinRadS = -18.0F;
constexpr float kSpeedMaxRadS = 18.0F;

volatile sig_atomic_t g_running = 1;

struct Options {
  std::string interfaceName = "vcan0";
  float currentRangeA = 10.0F;
  float baseCurrentA = 0.5F;
  float baseMotorTemperatureC = 25.0F;
  float baseMosTemperatureC = 30.0F;
  int printIntervalMs = 1000;
};

struct MotorState {
  bool seen = false;
  can_frame lastCommand{};
  can_frame lastAck{};
  float positionRad = 0.0F;
  float speedRadS = 0.0F;
  float currentA = 0.0F;
  float motorTemperatureC = 0.0F;
  float mosTemperatureC = 0.0F;
  uint64_t receivedCount = 0;
  uint64_t sentCount = 0;
  uint64_t previousReceivedCount = 0;
  std::chrono::steady_clock::time_point lastCommandTime{};
};

void handleSignal(int) { g_running = 0; }

void printUsage(const char *program) {
  std::fprintf(
      stderr,
      "Usage: %s [interface] [options]\n\n"
      "Options:\n"
      "  --current-range <A>  Feedback current range (default: 10.0)\n"
      "  --base-current <A>   Simulated current for Motor 1 (default: 0.5)\n"
      "  --motor-temp <C>     Base motor temperature (default: 25.0)\n"
      "  --mos-temp <C>       Base MOS temperature (default: 30.0)\n"
      "  --print-ms <ms>      Summary interval (default: 1000)\n"
      "  --help               Show this help\n\n"
      "Example:\n"
      "  %s vcan0 --current-range 10 --print-ms 1000\n",
      program, program);
}

bool parseFloat(const char *text, float &value) {
  char *end = nullptr;
  errno = 0;
  const float parsed = std::strtof(text, &end);
  if (errno != 0 || end == text || *end != '\0' || !std::isfinite(parsed)) {
    return false;
  }
  value = parsed;
  return true;
}

bool parseInt(const char *text, int &value) {
  char *end = nullptr;
  errno = 0;
  const long parsed = std::strtol(text, &end, 10);
  if (errno != 0 || end == text || *end != '\0' || parsed <= 0 ||
      parsed > 60000) {
    return false;
  }
  value = static_cast<int>(parsed);
  return true;
}

bool parseOptions(int argc, char **argv, Options &options) {
  int index = 1;
  if (index < argc && argv[index][0] != '-') {
    options.interfaceName = argv[index++];
  }

  while (index < argc) {
    const std::string argument = argv[index++];
    if (argument == "--help") {
      printUsage(argv[0]);
      std::exit(0);
    }
    if (index >= argc) {
      return false;
    }

    if (argument == "--current-range") {
      if (!parseFloat(argv[index++], options.currentRangeA) ||
          options.currentRangeA <= 0.0F) {
        return false;
      }
    } else if (argument == "--base-current") {
      if (!parseFloat(argv[index++], options.baseCurrentA)) {
        return false;
      }
    } else if (argument == "--motor-temp") {
      if (!parseFloat(argv[index++], options.baseMotorTemperatureC)) {
        return false;
      }
    } else if (argument == "--mos-temp") {
      if (!parseFloat(argv[index++], options.baseMosTemperatureC)) {
        return false;
      }
    } else if (argument == "--print-ms") {
      if (!parseInt(argv[index++], options.printIntervalMs)) {
        return false;
      }
    } else {
      return false;
    }
  }
  return true;
}

float clampFloat(float value, float minimum, float maximum) {
  return std::max(minimum, std::min(value, maximum));
}

float wrapPosition(float position) {
  constexpr float span = kPositionMaxRad - kPositionMinRad;
  while (position > kPositionMaxRad) {
    position -= span;
  }
  while (position < kPositionMinRad) {
    position += span;
  }
  return position;
}

float readFloatBe(const uint8_t *data) {
  const uint32_t bits = (static_cast<uint32_t>(data[0]) << 24) |
                        (static_cast<uint32_t>(data[1]) << 16) |
                        (static_cast<uint32_t>(data[2]) << 8) |
                        static_cast<uint32_t>(data[3]);
  float value = 0.0F;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

float readPackedPositionDegrees(const can_frame &frame) {
  uint64_t packed = 0;
  for (int index = 0; index < 8; ++index) {
    packed = (packed << 8) | frame.data[index];
  }
  const uint32_t positionBits =
      static_cast<uint32_t>((packed >> 29) & 0xFFFFFFFFULL);
  float positionDegrees = 0.0F;
  std::memcpy(&positionDegrees, &positionBits, sizeof(positionDegrees));
  return positionDegrees;
}

uint8_t encodeTemperature(float temperatureC) {
  const float raw = std::round((temperatureC * 2.0F) + 50.0F);
  return static_cast<uint8_t>(clampFloat(raw, 0.0F, 255.0F));
}

can_frame buildFeedbackFrame(int motorId, const MotorState &state,
                             float currentRangeA) {
  const uint16_t positionRaw = static_cast<uint16_t>(motor_float_to_uint(
      state.positionRad, kPositionMinRad, kPositionMaxRad, 16));
  const uint16_t speedRaw = static_cast<uint16_t>(
      motor_float_to_uint(state.speedRadS, kSpeedMinRadS, kSpeedMaxRadS, 12));
  const uint16_t currentRaw = static_cast<uint16_t>(
      motor_float_to_uint(state.currentA, -currentRangeA, currentRangeA, 12));

  can_frame ack{};
  ack.can_id = static_cast<canid_t>(motorId);
  ack.can_dlc = 8;
  ack.data[0] = 0x20; // Feedback type 1, error 0.
  ack.data[1] = static_cast<uint8_t>(positionRaw >> 8);
  ack.data[2] = static_cast<uint8_t>(positionRaw);
  ack.data[3] = static_cast<uint8_t>(speedRaw >> 4);
  ack.data[4] =
      static_cast<uint8_t>(((speedRaw & 0x0FU) << 4) | (currentRaw >> 8));
  ack.data[5] = static_cast<uint8_t>(currentRaw);
  ack.data[6] = encodeTemperature(state.motorTemperatureC);
  ack.data[7] = encodeTemperature(state.mosTemperatureC);
  return ack;
}

bool decodeCommand(const can_frame &frame, MotorState &state,
                   const Options &options) {
  const int motorId = static_cast<int>(frame.can_id & CAN_SFF_MASK);
  const bool speedMotor = (motorId % 2) != 0;
  const auto now = std::chrono::steady_clock::now();
  float elapsedSeconds = 0.0F;
  if (state.seen) {
    elapsedSeconds =
        std::chrono::duration<float>(now - state.lastCommandTime).count();
    elapsedSeconds = clampFloat(elapsedSeconds, 0.0F, 1.0F);
  }

  if (speedMotor) {
    if (frame.can_dlc != 7 || (frame.data[0] >> 5) != 2) {
      return false;
    }
    const uint8_t ackMode = static_cast<uint8_t>(frame.data[0] & 0x03U);
    if (ackMode != 1) {
      return false;
    }
    const float speedRpm = readFloatBe(frame.data + 1);
    if (!std::isfinite(speedRpm)) {
      return false;
    }
    state.speedRadS =
        clampFloat(speedRpm * kPi / 30.0F, kSpeedMinRadS, kSpeedMaxRadS);
    state.positionRad =
        wrapPosition(state.positionRad + (state.speedRadS * elapsedSeconds));
  } else {
    if (frame.can_dlc != 8 || (frame.data[0] >> 5) != 1) {
      return false;
    }
    const uint8_t ackMode = static_cast<uint8_t>(frame.data[7] & 0x03U);
    if (ackMode != 1) {
      return false;
    }
    const float positionDegrees = readPackedPositionDegrees(frame);
    if (!std::isfinite(positionDegrees)) {
      return false;
    }
    const float previousPosition = state.positionRad;
    state.positionRad = clampFloat(positionDegrees * kPi / 180.0F,
                                   kPositionMinRad, kPositionMaxRad);
    state.speedRadS = elapsedSeconds > 0.0001F
                          ? clampFloat((state.positionRad - previousPosition) /
                                           elapsedSeconds,
                                       kSpeedMinRadS, kSpeedMaxRadS)
                          : 0.0F;
  }

  state.currentA = clampFloat(options.baseCurrentA + (0.1F * motorId),
                              -options.currentRangeA, options.currentRangeA);
  state.motorTemperatureC = options.baseMotorTemperatureC + motorId;
  state.mosTemperatureC = options.baseMosTemperatureC + motorId;
  state.lastCommandTime = now;
  return true;
}

std::string frameToHex(const can_frame &frame) {
  char buffer[128] = {0};
  int offset = std::snprintf(buffer, sizeof(buffer), "0x%03X [%u]",
                             frame.can_id & CAN_SFF_MASK, frame.can_dlc);
  for (uint8_t index = 0; index < frame.can_dlc && offset > 0 &&
                          offset < static_cast<int>(sizeof(buffer));
       ++index) {
    offset += std::snprintf(buffer + offset,
                            sizeof(buffer) - static_cast<size_t>(offset),
                            " %02X", frame.data[index]);
  }
  return buffer;
}

void printSummary(std::array<MotorState, kMotorCount> &motors, int intervalMs) {
  std::printf("\n8-motor SocketCAN simulator (%d ms summary)\n", intervalMs);
  std::printf("%-5s %-6s %-31s %-31s %-10s %-10s %-9s %-8s %-8s\n", "ID",
              "RX/s", "last command", "last ACK", "pos(rad)", "spd(rad/s)",
              "cur(A)", "motorC", "mosC");

  for (int index = 0; index < kMotorCount; ++index) {
    MotorState &state = motors[static_cast<size_t>(index)];
    const uint64_t receivedSinceLast =
        state.receivedCount - state.previousReceivedCount;
    state.previousReceivedCount = state.receivedCount;
    const double rate = static_cast<double>(receivedSinceLast) * 1000.0 /
                        static_cast<double>(intervalMs);
    if (!state.seen) {
      std::printf("0x%03X %-6.1f %-31s %-31s %-10s %-10s %-9s %-8s %-8s\n",
                  index + 1, rate, "--", "--", "--", "--", "--", "--", "--");
      continue;
    }

    const std::string command = frameToHex(state.lastCommand);
    const std::string ack = frameToHex(state.lastAck);
    std::printf(
        "0x%03X %-6.1f %-31s %-31s %-10.4f %-10.4f %-9.3f %-8.1f %-8.1f\n",
        index + 1, rate, command.c_str(), ack.c_str(), state.positionRad,
        state.speedRadS, state.currentA, state.motorTemperatureC,
        state.mosTemperatureC);
  }
  std::fflush(stdout);
}

int openCanSocket(const Options &options) {
  const int socketFd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (socketFd < 0) {
    std::perror("socket");
    return -1;
  }

  const int receiveOwnMessages = 0;
  if (setsockopt(socketFd, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS,
                 &receiveOwnMessages, sizeof(receiveOwnMessages)) < 0) {
    std::perror("setsockopt(CAN_RAW_RECV_OWN_MSGS)");
    close(socketFd);
    return -1;
  }

  std::array<can_filter, kMotorCount> filters{};
  for (int index = 0; index < kMotorCount; ++index) {
    filters[static_cast<size_t>(index)].can_id =
        static_cast<canid_t>(index + 1);
    filters[static_cast<size_t>(index)].can_mask =
        CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
  }
  if (setsockopt(socketFd, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                 static_cast<socklen_t>(sizeof(filters))) < 0) {
    std::perror("setsockopt(CAN_RAW_FILTER)");
    close(socketFd);
    return -1;
  }

  ifreq request{};
  std::snprintf(request.ifr_name, IFNAMSIZ, "%s",
                options.interfaceName.c_str());
  if (ioctl(socketFd, SIOCGIFINDEX, &request) < 0) {
    std::perror("ioctl(SIOCGIFINDEX)");
    close(socketFd);
    return -1;
  }

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = request.ifr_ifindex;
  if (bind(socketFd, reinterpret_cast<sockaddr *>(&address), sizeof(address)) <
      0) {
    std::perror("bind");
    close(socketFd);
    return -1;
  }
  return socketFd;
}

} // namespace

int main(int argc, char **argv) {
  Options options;
  if (!parseOptions(argc, argv, options)) {
    printUsage(argv[0]);
    return 2;
  }

  signal(SIGINT, handleSignal);
  signal(SIGTERM, handleSignal);

  const int socketFd = openCanSocket(options);
  if (socketFd < 0) {
    return 1;
  }

  std::array<MotorState, kMotorCount> motors{};
  auto nextPrint = std::chrono::steady_clock::now() +
                   std::chrono::milliseconds(options.printIntervalMs);

  std::printf("Listening for Motor IDs 0x001-0x008 on %s\n",
              options.interfaceName.c_str());
  std::printf(
      "Odd IDs: speed commands; even IDs: position commands; ACK type: 1\n");
  std::printf("Press Ctrl+C to stop.\n");

  while (g_running != 0) {
    const auto now = std::chrono::steady_clock::now();
    const int timeoutMs = static_cast<int>(std::max<int64_t>(
        0,
        std::chrono::duration_cast<std::chrono::milliseconds>(nextPrint - now)
            .count()));
    pollfd descriptor{socketFd, POLLIN, 0};
    const int result = poll(&descriptor, 1, timeoutMs);
    if (result < 0) {
      if (errno == EINTR) {
        continue;
      }
      std::perror("poll");
      break;
    }

    if (result > 0 && (descriptor.revents & POLLIN) != 0) {
      can_frame command{};
      const ssize_t received = read(socketFd, &command, sizeof(command));
      if (received == static_cast<ssize_t>(sizeof(command)) &&
          (command.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) ==
              0) {
        const int motorId = static_cast<int>(command.can_id & CAN_SFF_MASK);
        if (motorId >= 1 && motorId <= kMotorCount) {
          MotorState &state = motors[static_cast<size_t>(motorId - 1)];
          if (decodeCommand(command, state, options)) {
            state.seen = true;
            state.lastCommand = command;
            ++state.receivedCount;
            const can_frame ack =
                buildFeedbackFrame(motorId, state, options.currentRangeA);
            const ssize_t written = write(socketFd, &ack, sizeof(ack));
            if (written == static_cast<ssize_t>(sizeof(ack))) {
              state.lastAck = ack;
              ++state.sentCount;
            } else {
              std::perror("write ACK");
            }
          }
        }
      }
    }

    if (std::chrono::steady_clock::now() >= nextPrint) {
      printSummary(motors, options.printIntervalMs);
      nextPrint = std::chrono::steady_clock::now() +
                  std::chrono::milliseconds(options.printIntervalMs);
    }
  }

  close(socketFd);
  std::printf("Simulator stopped.\n");
  return 0;
}
