#include <algorithm>
#include <cstdint>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "example_soem/net_compat.h"

extern "C" {
#include "example_soem/motor_layer.h"
}

namespace {

class NetworkRuntime {
public:
    NetworkRuntime() : ready_(encos_net_startup() != 0) {}
    ~NetworkRuntime() {
        if (ready_) {
            encos_net_cleanup();
        }
    }
    bool ready() const { return ready_; }

private:
    bool ready_;
};

struct TcpCanFrame {
    uint32_t can_id = 0;
    uint8_t flags = 0;
    uint8_t dlc = 0;
    uint8_t data[8] = {0};
};

enum class MotorCmd {
    Spd,
    Pos,
    Cur,
    Tor,
    Stop,
    Brake,
    ResetZero,
    GetParam,
    SetId,
    SetPos,
    SetAcc,
    SetKt,
    PVT,
};

bool buildMotorCommand(MotorCmd cmd, const MotorConfig& cfg, const std::vector<float>& args,
                       MotorPackMsg& packet) {
    switch (cmd) {
        case MotorCmd::Spd:
            if (args.size() < 3) {
                return false;
            }
            return motor_build_spd_control(&cfg, args[0], args[1], static_cast<uint8_t>(args[2]),
                                          &packet);
        case MotorCmd::Pos:
            if (args.size() < 4) {
                return false;
            }
            return motor_build_pos_control(&cfg, args[0], args[1], args[2],
                                          static_cast<uint8_t>(args[3]), &packet);
        case MotorCmd::Cur:
            if (args.size() < 2) {
                return false;
            }
            return motor_build_cur_control(&cfg, args[0], static_cast<uint8_t>(args[1]), &packet);
        case MotorCmd::Tor:
            if (args.size() < 2) {
                return false;
            }
            return motor_build_tor_control(&cfg, args[0], static_cast<uint8_t>(args[1]), &packet);
        case MotorCmd::Stop:
            if (args.size() < 3) {
                return false;
            }
            return motor_build_stop(&cfg, static_cast<MotorStopMode>(static_cast<int>(args[0])),
                                    args[1], static_cast<uint8_t>(args[2]), &packet);
        case MotorCmd::Brake:
            if (args.empty()) {
                return false;
            }
            return motor_build_brake(&cfg, args[0] != 0.0f, &packet);
        case MotorCmd::ResetZero:
            return motor_build_reset_zero_pos(&cfg, &packet);
        case MotorCmd::GetParam:
            if (args.empty()) {
                return false;
            }
            return motor_build_get_parameter(&cfg, static_cast<MotorParameter>(static_cast<int>(args[0])),
                                            &packet);
        case MotorCmd::SetId:
            if (args.empty()) {
                return false;
            }
            return motor_build_set_id(&cfg, static_cast<uint16_t>(args[0]), &packet);
        case MotorCmd::SetPos:
            if (args.empty()) {
                return false;
            }
            return motor_build_set_pos(&cfg, args[0], &packet);
        case MotorCmd::SetAcc:
            if (args.empty()) {
                return false;
            }
            return motor_build_set_acceleration(&cfg, args[0], true, &packet);
        case MotorCmd::SetKt:
            if (args.empty()) {
                return false;
            }
            return motor_build_set_kt(&cfg, args[0], &packet);
        case MotorCmd::PVT:
            if (args.size() < 5) {
                return false;
            }
            return motor_build_pvt_control(&cfg, args[0], args[1], args[2], args[3], args[4],
                                          &packet);
        default:
            return false;
    }
}

std::vector<uint8_t> encodeToGatewayFrame(const MotorPackMsg& packet) {
    std::vector<uint8_t> out;
    out.reserve(4 + 1 + 1 + 8);

    const uint32_t can_id = packet.id;
    out.push_back(static_cast<uint8_t>(can_id & 0xFF));
    out.push_back(static_cast<uint8_t>((can_id >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>((can_id >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((can_id >> 24) & 0xFF));
    out.push_back(packet.frame_flags);
    out.push_back(packet.len);

    for (uint8_t i = 0; i < packet.len; ++i) {
        out.push_back(packet.data[i]);
    }
    return out;
}

bool decodeFromGatewayFrame(const std::vector<uint8_t>& frame, MotorPackMsg& packet) {
    if (frame.size() < 6) {
        return false;
    }

    std::memset(&packet, 0, sizeof(packet));
    packet.id = static_cast<uint32_t>(frame[0]) |
                (static_cast<uint32_t>(frame[1]) << 8) |
                (static_cast<uint32_t>(frame[2]) << 16) |
                (static_cast<uint32_t>(frame[3]) << 24);
    packet.frame_flags = frame[4];
    packet.len = frame[5];

    const size_t payload_size = std::min<size_t>(static_cast<size_t>(packet.len), sizeof(packet.data));
    for (size_t i = 0; i < payload_size; ++i) {
        packet.data[i] = frame[6 + i];
    }
    return true;
}

enum class TransportMode {
    Udp,
    Tcp,
};

class UdpCanGateway {
public:
    UdpCanGateway() = default;
    ~UdpCanGateway() { close(); }

    bool connect(const std::string& host, int port) {
        close();

        fd_ = ::socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
        if (fd_ == ENCOS_INVALID_SOCKET) {
            return false;
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port));
        if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
            close();
            return false;
        }

        remote_addr_ = addr;

        if (::connect(fd_, reinterpret_cast<sockaddr*>(&remote_addr_), sizeof(remote_addr_)) < 0) {
            close();
            return false;
        }

        if (encos_set_receive_timeout(fd_, 1000) < 0) {
            close();
            return false;
        }

        return true;
    }

    bool sendFrame(const MotorPackMsg& packet) {
        const auto encoded = encodeToGatewayFrame(packet);
        const encos_ssize_t n = ::send(fd_, reinterpret_cast<const char*>(encoded.data()),
                                       static_cast<int>(encoded.size()), 0);
        return n == static_cast<encos_ssize_t>(encoded.size());
    }

    bool recvFrame(MotorPackMsg& packet) {
        uint8_t raw[64] = {0};
        const encos_ssize_t n = ::recv(fd_, reinterpret_cast<char*>(raw), sizeof(raw), 0);
        if (n <= 0) {
            return false;
        }

        std::vector<uint8_t> frame(raw, raw + n);
        return decodeFromGatewayFrame(frame, packet);
    }

    void close() {
        if (fd_ != ENCOS_INVALID_SOCKET) {
            encos_close_socket(fd_);
            fd_ = ENCOS_INVALID_SOCKET;
        }
    }

private:
    encos_socket_t fd_ = ENCOS_INVALID_SOCKET;
    sockaddr_in remote_addr_{};
};

class TcpSocketGateway {
public:
    TcpSocketGateway() = default;
    ~TcpSocketGateway() { close(); }

    bool connect(const std::string& host, int port) {
        close();

        fd_ = ::socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
        if (fd_ == ENCOS_INVALID_SOCKET) {
            return false;
        }

        sockaddr_in addr{};
        addr.sin_family = AF_INET;
        addr.sin_port = htons(static_cast<uint16_t>(port));
        if (inet_pton(AF_INET, host.c_str(), &addr.sin_addr) != 1) {
            close();
            return false;
        }

        if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
            close();
            return false;
        }

        if (encos_set_receive_timeout(fd_, 1000) < 0) {
            close();
            return false;
        }

        return true;
    }

    bool sendFrame(const MotorPackMsg& packet) {
        const auto encoded = encodeToGatewayFrame(packet);
        size_t sent = 0;
        while (sent < encoded.size()) {
            const encos_ssize_t n = ::send(
                fd_, reinterpret_cast<const char*>(encoded.data() + sent),
                static_cast<int>(encoded.size() - sent), 0);
            if (n <= 0) {
                return false;
            }
            sent += static_cast<size_t>(n);
        }
        return true;
    }

    bool recvFrame(MotorPackMsg& packet) {
        uint8_t raw[64] = {0};
        const encos_ssize_t n = ::recv(fd_, reinterpret_cast<char*>(raw), sizeof(raw), 0);
        if (n <= 0) {
            return false;
        }

        std::vector<uint8_t> frame(raw, raw + n);
        return decodeFromGatewayFrame(frame, packet);
    }

    void close() {
        if (fd_ != ENCOS_INVALID_SOCKET) {
            encos_close_socket(fd_);
            fd_ = ENCOS_INVALID_SOCKET;
        }
    }

private:
    encos_socket_t fd_ = ENCOS_INVALID_SOCKET;
};

bool sendFrameByTransport(TransportMode mode, const std::string& host, int port,
                          const MotorPackMsg& packet) {
    if (mode == TransportMode::Udp) {
        UdpCanGateway gateway;
        if (!gateway.connect(host, port)) {
            return false;
        }
        return gateway.sendFrame(packet);
    }

    TcpSocketGateway gateway;
    if (!gateway.connect(host, port)) {
        return false;
    }
    return gateway.sendFrame(packet);
}

bool recvFrameByTransport(TransportMode mode, const std::string& host, int port,
                          MotorPackMsg& packet) {
    if (mode == TransportMode::Udp) {
        UdpCanGateway gateway;
        if (!gateway.connect(host, port)) {
            return false;
        }
        return gateway.recvFrame(packet);
    }

    TcpSocketGateway gateway;
    if (!gateway.connect(host, port)) {
        return false;
    }
    return gateway.recvFrame(packet);
}

void printUsage(const char* program) {
    std::cerr << "Usage:\n"
              << "  " << program
              << " [udp|tcp] <ip> <port> <slaveId> <busId> <motorId> <model> <cmd> [args...]\n\n"
              << "Transport modes:\n"
              << "  udp  = DM-LinkX / Cannelloni UDP-to-SocketCAN bridge\n"
              << "  tcp  = plain TCP socket to another computer\n\n"
              << "Commands:\n"
              << "  spd <speed_rad_s> <current_a> <feedback>\n"
              << "  pos <position_rad> <speed_rad_s> <current_a> <feedback>\n"
              << "  cur <current_a> <feedback>\n"
              << "  tor <torque_nm> <feedback>\n"
              << "  stop <mode> <current_a> <feedback>\n"
              << "  brake <0|1>\n"
              << "  reset-zero\n"
              << "  get-param <parameter_id>\n"
              << "  set-id <new_id>\n"
              << "  set-pos <position_rad>\n"
              << "  set-acc <acceleration>\n"
              << "  set-kt <kt>\n"
              << "  pvt <kp> <kd> <position_rad> <speed_rad_s> <torque_nm>\n";
}

MotorCmd parseCommandName(const std::string& name) {
    if (name == "spd") {
        return MotorCmd::Spd;
    }
    if (name == "pos") {
        return MotorCmd::Pos;
    }
    if (name == "cur") {
        return MotorCmd::Cur;
    }
    if (name == "tor") {
        return MotorCmd::Tor;
    }
    if (name == "stop") {
        return MotorCmd::Stop;
    }
    if (name == "brake") {
        return MotorCmd::Brake;
    }
    if (name == "reset-zero") {
        return MotorCmd::ResetZero;
    }
    if (name == "get-param") {
        return MotorCmd::GetParam;
    }
    if (name == "set-id") {
        return MotorCmd::SetId;
    }
    if (name == "set-pos") {
        return MotorCmd::SetPos;
    }
    if (name == "set-acc") {
        return MotorCmd::SetAcc;
    }
    if (name == "set-kt") {
        return MotorCmd::SetKt;
    }
    if (name == "pvt") {
        return MotorCmd::PVT;
    }
    throw std::runtime_error("unsupported command: " + name);
}

std::vector<float> parseArgs(const std::vector<std::string>& text_args) {
    std::vector<float> values;
    values.reserve(text_args.size());
    for (const auto& text : text_args) {
        values.push_back(std::stof(text));
    }
    return values;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 8) {
        printUsage(argv[0]);
        return 2;
    }

    NetworkRuntime network;
    if (!network.ready()) {
        encos_socket_error("network startup");
        return 1;
    }

    try {
        int arg_index = 1;
        TransportMode mode = TransportMode::Udp;

        if (std::string(argv[1]) == "udp" || std::string(argv[1]) == "tcp") {
            mode = (std::string(argv[1]) == "udp") ? TransportMode::Udp : TransportMode::Tcp;
            arg_index = 2;
        }

        const std::string host = argv[arg_index];
        const int port = std::stoi(argv[arg_index + 1]);

        const MotorConfig cfg = motor_config_make(
            static_cast<uint16_t>(std::stoi(argv[arg_index + 2])),
            static_cast<uint16_t>(std::stoi(argv[arg_index + 3])),
            static_cast<uint16_t>(std::stoi(argv[arg_index + 4])),
            static_cast<MotorModel>(std::stoi(argv[arg_index + 5])));

        const std::string cmd_name = argv[arg_index + 6];
        std::vector<std::string> raw_args;
        for (int i = arg_index + 7; i < argc; ++i) {
            raw_args.emplace_back(argv[i]);
        }

        const MotorCmd command = parseCommandName(cmd_name);
        const std::vector<float> args = parseArgs(raw_args);

        MotorPackMsg packet{};
        if (!buildMotorCommand(command, cfg, args, packet)) {
            std::cerr << "failed to build packet for command: " << cmd_name << "\n";
            return 1;
        }

        if (!sendFrameByTransport(mode, host, port, packet)) {
            if (mode == TransportMode::Udp) {
                std::cerr << "failed to send packet to DM-LinkX UDP gateway at " << host << ':' << port
                          << "\n";
            } else {
                std::cerr << "failed to send packet to TCP server at " << host << ':' << port << "\n";
            }
            return 1;
        }

        std::cout << "sent via " << ((mode == TransportMode::Udp) ? "UDP" : "TCP")
                  << ": id=" << packet.id << " flags=0x" << std::hex
                  << static_cast<int>(packet.frame_flags) << " len=" << std::dec
                  << static_cast<int>(packet.len) << "\n";

        MotorPackMsg reply{};
        if (recvFrameByTransport(mode, host, port, reply)) {
            std::cout << "received reply\n";
            motor_print_received_packet(&reply, 30.0f);
        } else {
            std::cout << "no reply received before timeout\n";
        }

        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "error: " << ex.what() << "\n";
        printUsage(argv[0]);
        return 1;
    }
}
