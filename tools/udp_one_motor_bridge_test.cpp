#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "example_soem/net_compat.h"

extern "C" {
#include "example_soem/motor_layer.h"
}

namespace {

std::vector<uint8_t> encodeGatewayPayload(const MotorPackMsg& packet) {
    std::vector<uint8_t> out;
    out.reserve(4 + 1 + 1 + 8);

    out.push_back(static_cast<uint8_t>(packet.id & 0xFFu));
    out.push_back(static_cast<uint8_t>((packet.id >> 8) & 0xFFu));
    out.push_back(static_cast<uint8_t>((packet.id >> 16) & 0xFFu));
    out.push_back(static_cast<uint8_t>((packet.id >> 24) & 0xFFu));
    out.push_back(packet.frame_flags);
    out.push_back(packet.len);

    for (uint8_t i = 0; i < packet.len; ++i) {
        out.push_back(packet.data[i]);
    }

    return out;
}

void printHex(const std::vector<uint8_t>& bytes) {
    for (size_t i = 0; i < bytes.size(); ++i) {
        std::cout << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(bytes[i]);
        if (i + 1 < bytes.size()) {
            std::cout << ' ';
        }
    }
    std::cout << std::dec << '\n';
}

bool buildMotorPacket(const std::string& cmd, float p1, float p2, float p3, float p4,
                      MotorPackMsg& packet) {
    MotorConfig cfg = motor_config_make(1, 0, 1, MOTOR_MODEL_EC_A2806_P2);
    std::memset(&packet, 0, sizeof(packet));

    if (cmd == "spd") {
        return motor_build_spd_control(&cfg, p1, p2, static_cast<uint8_t>(p3), &packet);
    }
    if (cmd == "pos") {
        return motor_build_pos_control(&cfg, p1, p2, p3, static_cast<uint8_t>(p4), &packet);
    }
    if (cmd == "cur") {
        return motor_build_cur_control(&cfg, p1, static_cast<uint8_t>(p2), &packet);
    }
    if (cmd == "tor") {
        return motor_build_tor_control(&cfg, p1, static_cast<uint8_t>(p2), &packet);
    }
    if (cmd == "stop") {
        return motor_build_stop(&cfg, MOTOR_STOP_FULL_BRAKE, p1, static_cast<uint8_t>(p2), &packet);
    }
    if (cmd == "set-pos") {
        return motor_build_set_pos(&cfg, p1, &packet);
    }
    if (cmd == "set-id") {
        return motor_build_set_id(&cfg, static_cast<uint16_t>(p1), &packet);
    }

    std::cerr << "Unsupported command: " << cmd << "\n";
    std::cerr << "Supported: spd pos cur tor stop set-pos set-id\n";
    return false;
}

void printUsage(const char* program) {
    std::cerr << "Usage: " << program << " <target_ip> <port> [command] [p1] [p2] [p3] [p4]\n\n";
    std::cerr << "Examples:\n";
    std::cerr << "  " << program << " 192.168.1.50 5000 spd 1.0 1.5 0 0\n";
    std::cerr << "  " << program << " 192.168.1.50 5000 cur 2.0 1\n";
    std::cerr << "  " << program << " 192.168.1.50 5000 set-id 7\n";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        printUsage(argv[0]);
        return 2;
    }

    const std::string targetIp = argv[1];
    const int port = std::atoi(argv[2]);
    const std::string command = (argc >= 4) ? argv[3] : "spd";

    float p1 = 1.0f;
    float p2 = 1.5f;
    float p3 = 0.0f;
    float p4 = 0.0f;

    if (argc >= 5) {
        p1 = std::stof(argv[4]);
    }
    if (argc >= 6) {
        p2 = std::stof(argv[5]);
    }
    if (argc >= 7) {
        p3 = std::stof(argv[6]);
    }
    if (argc >= 8) {
        p4 = std::stof(argv[7]);
    }

    MotorPackMsg packet{};
    if (!buildMotorPacket(command, p1, p2, p3, p4, packet)) {
        return 3;
    }

    const std::vector<uint8_t> payload = encodeGatewayPayload(packet);

    std::cout << "Bridge target: " << targetIp << ':' << port << '\n';
    std::cout << "Command: " << command << " p1=" << p1 << " p2=" << p2
              << " p3=" << p3 << " p4=" << p4 << '\n';
    std::cout << "packet.id=" << packet.id << " flags=0x" << std::hex << std::setw(2)
              << std::setfill('0') << static_cast<int>(packet.frame_flags) << std::dec
              << " len=" << static_cast<int>(packet.len) << '\n';
    std::cout << "UDP payload bytes: ";
    printHex(payload);

    if (!encos_net_startup()) {
        encos_socket_error("network startup");
        return 4;
    }

    const encos_socket_t sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock == ENCOS_INVALID_SOCKET) {
        encos_socket_error("socket");
        encos_net_cleanup();
        return 4;
    }

    sockaddr_in addr{};
    std::memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;
    addr.sin_port = htons(static_cast<uint16_t>(port));
    if (inet_pton(AF_INET, targetIp.c_str(), &addr.sin_addr) != 1) {
        std::cerr << "Invalid target IP: " << targetIp << '\n';
        encos_close_socket(sock);
        encos_net_cleanup();
        return 5;
    }

    const encos_ssize_t sent =
        sendto(sock, reinterpret_cast<const char*>(payload.data()),
               static_cast<int>(payload.size()), 0,
               reinterpret_cast<const sockaddr*>(&addr), sizeof(addr));
    if (sent < 0) {
        encos_socket_error("sendto");
        encos_close_socket(sock);
        encos_net_cleanup();
        return 6;
    }

    std::cout << "Sent " << sent << " bytes to UDP bridge.\n";
    encos_close_socket(sock);
    encos_net_cleanup();
    return 0;
}
