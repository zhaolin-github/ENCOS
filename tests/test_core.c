#include <math.h>
#include <stdio.h>
#include <string.h>

#include "example_soem/demo_timing.h"
#include "example_soem/ethercat_layer.h"
#include "example_soem/motor_layer.h"

#define CHECK(expr)                                                            \
  do {                                                                         \
    if (!(expr)) {                                                             \
      fprintf(stderr, "%s:%d: check failed: %s\n", __FILE__, __LINE__, #expr); \
      return 1;                                                                \
    }                                                                          \
  } while (0)

static int test_pdo_identification(void) {
  EcSlavePdoLayout layout;

  CHECK(ec_identify_pdo_layout(86, &layout));
  CHECK(layout.format == EC_SLAVE_FORMAT_CLASSIC_CAN_2_BUS);
  CHECK(layout.bus_count == 2);
  CHECK(layout.slots_per_bus == 3);
  CHECK(layout.motor_offset == 2);

  CHECK(ec_identify_pdo_layout(336, &layout));
  CHECK(layout.format == EC_SLAVE_FORMAT_CAN_FD_3_BUS);
  CHECK(layout.bus_count == 3);
  CHECK(layout.slots_per_bus == 8);
  CHECK(layout.motor_offset == 0);

  CHECK(ec_identify_pdo_layout(896, &layout));
  CHECK(layout.format == EC_SLAVE_FORMAT_CAN_FD_8_BUS);
  CHECK(layout.bus_count == 8);
  CHECK(layout.slots_per_bus == 8);

  CHECK(!ec_identify_pdo_layout(340, &layout));
  return 0;
}

static int test_validate_and_offset(void) {
  EcLayoutTable table = {0};
  CHECK(ec_identify_pdo_layout(336, &table.slaves[0]));
  table.slave_count = 1;

  CHECK(ec_validate_target(&table, 0, 2, 7));
  CHECK(!ec_validate_target(&table, 1, 0, 0));
  CHECK(!ec_validate_target(&table, 0, 3, 0));
  CHECK(!ec_validate_target(&table, 0, 0, 8));

  CHECK(ec_motor_slot_offset(&table.slaves[0], 2, 7) ==
        23 * sizeof(MotorPackMsg));
  return 0;
}

static int test_write_packet_offset(void) {
  EcSlavePdoLayout layout;
  uint8_t output[336] = {0};
  MotorPackMsg packet = {0};
  MotorPackMsg copied = {0};

  CHECK(ec_identify_pdo_layout(sizeof(output), &layout));
  packet.id = 0x123;
  packet.frame_flags = MOTOR_CAN_FLAG_FD_MASK;
  packet.len = 3;
  packet.data[0] = 0x60;
  packet.data[1] = 0x12;
  packet.data[2] = 0x34;

  CHECK(ec_write_motor_packet(output, &layout, 1, 2, &packet));
  memcpy(&copied, output + ec_motor_slot_offset(&layout, 1, 2), sizeof(copied));
  CHECK(copied.id == packet.id);
  CHECK(copied.frame_flags == packet.frame_flags);
  CHECK(copied.len == packet.len);
  CHECK(copied.data[0] == packet.data[0]);
  return 0;
}

static int test_motor_encoding(void) {
  MotorConfig config = {0};
  MotorPackMsg packet = {0};
  config.motorId = 1;
  config.flag = MOTOR_CAN_FLAG_FD_MASK;
  config.model = MOTOR_MODEL_EC_A4310_P2;

  CHECK(motor_build_spd_control(&config, 1.0f, 2.0f, 3, &packet));
  CHECK(packet.id == 1);
  CHECK(packet.frame_flags == MOTOR_CAN_FLAG_FD_MASK);
  CHECK(packet.len == 7);
  CHECK(packet.data[0] == (uint8_t)(0x40 | 3));

  CHECK(motor_build_reset_zero_pos(&config, &packet));
  CHECK(packet.id == 0x7FF);
  CHECK(packet.len == 6);
  CHECK(packet.data[0] == 0);
  CHECK(packet.data[1] == 1);
  CHECK(packet.data[3] == 0x03);
  CHECK(packet.data[4] == 0);
  CHECK(packet.data[5] == 0);

  CHECK(motor_build_get_parameter(&config, MOTOR_PARAM_ACCELERATION, &packet));
  CHECK(packet.id == 1);
  CHECK(packet.len == 2);
  CHECK(packet.data[0] == (uint8_t)((0x07 << 5) | 1));
  CHECK(packet.data[1] == MOTOR_PARAM_ACCELERATION);
  return 0;
}

static int test_decode_query_feedback_float(void) {
  MotorPackMsg packet = {0};
  MotorQueryFeedbackFloat feedback = {0};

  packet.id = 1;
  packet.len = 6;
  packet.data[0] = 0xA0;
  packet.data[1] = MOTOR_PARAM_SPEED;
  packet.data[2] = 0x42;
  packet.data[3] = 0x06;
  packet.data[4] = 0x66;
  packet.data[5] = 0x66;

  CHECK(motor_decode_query_feedback_float(&packet, &feedback));
  CHECK(feedback.message_type == 5);
  CHECK(feedback.error == 0);
  CHECK(feedback.query_code == MOTOR_PARAM_SPEED);
  CHECK(fabsf(feedback.value - 33.6f) < 0.0001f);

  packet.len = 5;
  CHECK(!motor_decode_query_feedback_float(&packet, &feedback));
  return 0;
}

static int test_motor_config_flags(void) {
  const MotorConfig classic =
      motor_config_make(0, 2, 11, MOTOR_MODEL_EC_A4310_P2);
  const MotorConfig canfd = motor_config_make_with_flags(
      0, 2, 11, MOTOR_MODEL_EC_A4310_P2, true, true);

  CHECK(classic.slaveId == 0);
  CHECK(classic.busId == 2);
  CHECK(classic.motorId == 11);
  CHECK(classic.flag == 0);
  CHECK(canfd.flag == (MOTOR_CAN_FLAG_EFF | MOTOR_CAN_FLAG_FD_MASK));
  return 0;
}

static int test_decode_feedback1(void) {
  MotorPackMsg packet = {0};
  MotorFeedback1 feedback = {0};

  packet.id = 3;
  packet.len = 8;
  packet.data[0] = 0x23;
  packet.data[1] = 0xFF;
  packet.data[2] = 0xFF;
  packet.data[3] = 0xFF;
  packet.data[4] = 0xFF;
  packet.data[5] = 0xFF;
  packet.data[6] = 90;
  packet.data[7] = 110;

  CHECK(motor_decode_feedback1(&packet, 30.0f, &feedback));
  CHECK(feedback.message_type == 1);
  CHECK(feedback.error == 3);
  CHECK(feedback.position_raw == 65535);
  CHECK(feedback.speed_raw == 4095);
  CHECK(feedback.current_raw == 4095);
  CHECK(fabsf(feedback.position_rad - 12.5f) < 0.0001f);
  CHECK(fabsf(feedback.speed_rad_s - 18.0f) < 0.0001f);
  CHECK(fabsf(feedback.current_a - 30.0f) < 0.0001f);
  CHECK(fabsf(feedback.motor_temperature_c - 20.0f) < 0.0001f);
  CHECK(fabsf(feedback.mos_temperature_c - 30.0f) < 0.0001f);

  packet.len = 7;
  CHECK(!motor_decode_feedback1(&packet, 30.0f, &feedback));
  return 0;
}

static int test_time_add_us(void) {
  DemoTimePoint point = {3999900000LL};

  demo_time_add_us(&point, 250);

  CHECK(point.nanoseconds == 4000150000LL);
  return 0;
}

int main(void) {
  CHECK(sizeof(MotorPackMsg) == 14);
  CHECK(test_pdo_identification() == 0);
  CHECK(test_validate_and_offset() == 0);
  CHECK(test_write_packet_offset() == 0);
  CHECK(test_motor_encoding() == 0);
  CHECK(test_decode_query_feedback_float() == 0);
  CHECK(test_motor_config_flags() == 0);
  CHECK(test_decode_feedback1() == 0);
  CHECK(test_time_add_us() == 0);
  return 0;
}
