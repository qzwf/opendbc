// BYD ATTO3 Safety Implementation
// Bus layout: 0=car chassis CAN, 1=radar/camera private, 2=camera ADAS CAN
// OpenPilot sends STEERING_MODULE_ADAS on bus 0 directly to EPS.
// Camera's version on bus 2 is blocked via byd_fwd_hook only when controls_allowed.
// All other camera ADAS messages (ACC, AEB, etc.) always forward — car retains priority.

#pragma once

#include "opendbc/safety/declarations.h"

// CAN message IDs (verified against byd_general.dbc)
#define BYD_STEERING_TORQUE      0x1FCU  // 508  - MAIN_TORQUE from EPS, bus 0
#define BYD_STEER_MODULE_2       0x11FU  // 287  - STEER_ANGLE_2, bus 0
#define BYD_PEDAL                0x342U  // 834  - GAS_PEDAL/BRAKE_PEDAL, bus 0
#define BYD_DRIVE_STATE          0x242U  // 578  - BRAKE_PRESSED/GEAR, bus 0
#define BYD_STEERING_MODULE_ADAS 0x1E2U  // 482  - LKAS steering command, camera on bus 2
#define BYD_LKAS_HUD_ADAS        0x316U  // 790  - LKAS HUD, camera on bus 2
#define BYD_ACC_HUD_ADAS         0x32DU  // 813  - ACC status (ACC_ON1/ON2), camera on bus 2

static safety_config byd_init(uint16_t param) {
  // Messages OpenPilot is allowed to send:
  // - STEERING_MODULE_ADAS on bus 0 (direct to EPS, camera's version blocked via fwd_hook)
  // - LKAS_HUD_ADAS on bus 2 (competes with camera for display update — acceptable)
  static const CanMsg BYD_TX_MSGS[] = {
    {(int)BYD_STEERING_MODULE_ADAS, 0, 8, .check_relay = false},
    {(int)BYD_LKAS_HUD_ADAS,        2, 8, .check_relay = false},
  };

  // Messages we must monitor to maintain safety state
  static RxCheck byd_rx_checks[] = {
    {.msg = {{(int)BYD_STEERING_TORQUE,      0, 8, 100U, .ignore_checksum=true, .ignore_counter=true, .ignore_quality_flag=true}, {0}, {0}}},
    {.msg = {{(int)BYD_STEER_MODULE_2,       0, 5, 100U, .ignore_checksum=true, .ignore_counter=true, .ignore_quality_flag=true}, {0}, {0}}},
    {.msg = {{(int)BYD_PEDAL,                0, 8,  50U, .ignore_checksum=true, .ignore_counter=true, .ignore_quality_flag=true}, {0}, {0}}},
    {.msg = {{(int)BYD_DRIVE_STATE,          0, 8,  50U, .ignore_checksum=true, .ignore_counter=true, .ignore_quality_flag=true}, {0}, {0}}},
    {.msg = {{(int)BYD_ACC_HUD_ADAS,         2, 8,  50U, .ignore_checksum=true, .ignore_counter=true, .ignore_quality_flag=true}, {0}, {0}}},
  };

  SAFETY_UNUSED(param);
  return BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS);
}

static void byd_rx_hook(const CANPacket_t *to_push) {
  int bus = (int)to_push->bus;
  uint32_t addr = to_push->addr;

  // Driver steering torque — used for driver-override detection
  // MAIN_TORQUE: bits 0-15 @1- (LE signed, factor 0.1 Nm)
  if ((bus == 0) && (addr == BYD_STEERING_TORQUE)) {
    int torque_new = (int)GET_BYTES(to_push, 0, 2);
    torque_new = to_signed(torque_new, 16);
    update_sample(&torque_driver, torque_new);
  }

  // Gas pedal — GAS_PEDAL: bits 0-7 @1+ (LE, raw 0-255, factor 0.01)
  if ((bus == 0) && (addr == BYD_PEDAL)) {
    gas_pressed = to_push->data[0] > 10U;
  }

  // Brake pedal — BRAKE_PRESSED: 37|1@0+ (1-bit Motorola, DBC bit == Intel bit)
  if ((bus == 0) && (addr == BYD_DRIVE_STATE)) {
    brake_pressed = GET_BIT(to_push, 37U);
  }

  // ACC engagement — sets controls_allowed on rising edge of ACC being on
  // ACC_ON1: 22|1@0+ (1-bit Motorola, DBC bit == Intel bit)
  // ACC_ON2: 20|1@0+ (1-bit Motorola, DBC bit == Intel bit)
  if ((bus == 2) && (addr == BYD_ACC_HUD_ADAS)) {
    bool cruise_engaged = GET_BIT(to_push, 22U) && GET_BIT(to_push, 20U);
    pcm_cruise_check(cruise_engaged);
  }
}

static bool byd_tx_hook(const CANPacket_t *to_send) {
  // STEER_ANGLE DBC factor 0.1 deg → raw 1000 = 100 physical degrees.
  // Angle control: EPS targets absolute steering wheel position.
  // 100 degrees covers urban curves up to ~80 km/h.
  // max_rate_up=10 raw/msg at 50 Hz → 50 deg/s peak ramp.
  // max_rt_delta=125 raw / 250ms matches 50 deg/s (10 raw/msg × 12.5 msgs).
  // max_rate_down=20 raw/msg matches carcontroller STEER_DELTA_DOWN=1.0 deg/call × 2 calls/msg.
  const TorqueSteeringLimits BYD_STEERING_LIMITS = {
    .max_torque = 1000,
    .max_rate_up = 10,
    .max_rate_down = 20,
    .max_rt_delta = 125,
    .driver_torque_allowance = 100,
    .driver_torque_multiplier = 3,
    .type = TorqueDriverLimited,
  };

  bool tx = true;
  int bus = (int)to_send->bus;
  uint32_t addr = to_send->addr;

  // Steering control sent on bus 0 (directly to EPS)
  // STEER_ANGLE: 24|16@1- (LE signed, bytes 3-4)
  // STEER_REQ: 21|1@0+ (1-bit Motorola, DBC bit == Intel bit)
  if ((bus == 0) && (addr == BYD_STEERING_MODULE_ADAS)) {
    int desired_torque = (int)GET_BYTES(to_send, 3, 2);
    desired_torque = to_signed(desired_torque, 16);
    int steer_req = GET_BIT(to_send, 21U) ? 1 : 0;

    if (steer_torque_cmd_checks(desired_torque, steer_req, BYD_STEERING_LIMITS)) {
      tx = false;
    }
  }

  return tx;
}

// Forward all camera messages to car EXCEPT STEERING_MODULE_ADAS when OpenPilot is controlling.
// Camera's ACC, AEB, and all safety ADAS always reach the car (car retains full priority).
// Camera's LKAS is superseded only when controls_allowed (OpenPilot actively steering).
static bool byd_fwd_hook(int bus_num, int addr) {
  if ((bus_num == 2) && ((uint32_t)addr == BYD_STEERING_MODULE_ADAS) && controls_allowed) {
    return true;  // block camera's LKAS — OpenPilot sends on bus 0 directly
  }
  return false;  // forward all other messages normally
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
  .fwd = byd_fwd_hook,
};
