#pragma once

#include "opendbc/safety/safety_declarations.h"

#define BYD_LIMITS(steer, rate_up, rate_down) { \
  .max_torque = (steer), \
  .max_rate_up = (rate_up), \
  .max_rate_down = (rate_down), \
  .max_rt_delta = 375, \
  .driver_torque_allowance = 68, \
  .driver_torque_multiplier = 3, \
  .type = TorqueDriverLimited, \
  .min_valid_request_frames = 1, \
  .max_invalid_request_frames = 1, \
  .min_valid_request_rt_interval = 250000,  /* 250ms */ \
  .has_steer_req_tolerance = false, \
}

extern const LongitudinalLimits BYD_LONG_LIMITS;
const LongitudinalLimits BYD_LONG_LIMITS = {
  .max_accel = 200,   // 1/100 m/s2 (2.0 m/s2)
  .min_accel = -350,  // 1/100 m/s2 (-3.5 m/s2)
};

// BYD TX messages that openpilot is allowed to send
#define BYD_TX_MSGS \
  {0x1E2, 0, 8, .check_relay = true},   /* STEERING_MODULE_ADAS Bus 0 */ \
  {0x316, 0, 8, .check_relay = true},   /* LKAS_HUD_ADAS Bus 0 */ \
  {0x32D, 0, 8, .check_relay = true},   /* ACC_HUD_ADAS Bus 0 */ \
  {0x32E, 0, 8, .check_relay = true},   /* ACC_CMD Bus 0 */ \

// BYD RX checks for critical safety messages
#define BYD_RX_CHECKS \
  /* Steering wheel torque from driver */ \
  {.msg = {{0x1FC, 0, 8, .max_counter = 15U, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}}, \
  /* Pedal state and gas/brake */ \
  {.msg = {{0x342, 0, 8, .max_counter = 15U, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}}, \
  /* Vehicle speed */ \
  {.msg = {{0x220, 0, 8, .max_counter = 15U, .ignore_quality_flag = true, .frequency = 100U}, { 0 }, { 0 }}}, \
  /* Cruise control buttons */ \
  {.msg = {{0x3B0, 0, 8, .max_counter = 15U, .ignore_quality_flag = true, .frequency = 50U}, { 0 }, { 0 }}}, \

static const CanMsg BYD_TX_MSGS_LIST[] = {
  BYD_TX_MSGS
};

static uint8_t byd_get_counter(const CANPacket_t *msg) {
  int addr = GET_ADDR(msg);
  
  uint8_t cnt = 0;
  if (addr == 0x1FC) {          // STEERING_TORQUE
    cnt = (GET_BYTE(msg, 6) >> 4) & 0xFU;
  } else if (addr == 0x1E2) {   // STEERING_MODULE_ADAS
    cnt = (GET_BYTE(msg, 6) >> 4) & 0xFU;
  } else if (addr == 0x342) {   // PEDAL
    cnt = (GET_BYTE(msg, 6) >> 4) & 0xFU;
  } else if (addr == 0x220) {   // Vehicle speed (example)
    cnt = (GET_BYTE(msg, 7) >> 4) & 0xFU;
  } else if (addr == 0x3B0) {   // PCM_BUTTONS
    cnt = (GET_BYTE(msg, 6) >> 4) & 0xFU;
  } else {
    // Unknown message - no counter
  }
  return cnt;
}

static uint32_t byd_get_checksum(const CANPacket_t *msg) {
  int addr = GET_ADDR(msg);
  
  uint8_t chksum = 0;
  if (addr == 0x1FC) {          // STEERING_TORQUE
    chksum = GET_BYTE(msg, 7) & 0xFFU;
  } else if (addr == 0x1E2) {   // STEERING_MODULE_ADAS
    chksum = GET_BYTE(msg, 7) & 0xFFU;
  } else if (addr == 0x342) {   // PEDAL
    chksum = GET_BYTE(msg, 7) & 0xFFU;
  } else if (addr == 0x220) {   // Vehicle speed
    chksum = GET_BYTE(msg, 7) & 0xFFU;
  } else if (addr == 0x3B0) {   // PCM_BUTTONS
    chksum = GET_BYTE(msg, 7) & 0xFFU;
  } else {
    // Unknown message - no checksum
  }
  return chksum;
}

static uint32_t byd_compute_checksum(const CANPacket_t *msg) {
  int addr = GET_ADDR(msg);
  
  uint8_t chksum = 0;
  // Simple sum of nibbles checksum (similar to Hyundai)
  for (int i = 0; i < 8; i++) {
    if ((addr == 0x1E2) && (i == 7)) {
      continue; // exclude checksum byte
    }
    uint8_t b = GET_BYTE(msg, i);
    if (((addr == 0x1FC) && (i == 7)) || 
        ((addr == 0x1E2) && (i == 7)) ||
        ((addr == 0x342) && (i == 7))) {
      b &= 0xF0U; // remove checksum nibble if in same byte
    }
    chksum += (b % 16U) + (b / 16U);
  }
  chksum = (16U - (chksum % 16U)) % 16U;
  
  return chksum;
}

static void byd_rx_hook(const CANPacket_t *msg) {
  int bus = GET_BUS(msg);
  int addr = GET_ADDR(msg);

  if (bus == 0) {
    // Monitor steering torque from driver (STEERING_TORQUE message)
    if (addr == 0x1FC) {
      int torque_driver_new = GET_BYTES(msg, 0, 2) & 0xFFFFU;
      // Convert to signed value (assuming -2048 to +2047 range)
      torque_driver_new = to_signed(torque_driver_new, 16);
      torque_driver_new = torque_driver_new / 10; // Scale factor from DBC (0.1)
      
      // Update array of samples
      update_sample(&torque_driver, torque_driver_new);
    }

    // Monitor gas and brake pedals (PEDAL message)
    if (addr == 0x342) {
      // GAS_PEDAL at bit 0, BRAKE_PEDAL at bit 8 (from DBC)
      int gas_pedal_raw = GET_BYTE(msg, 0);
      int brake_pedal_raw = GET_BYTE(msg, 1);
      
      // Scale by 0.01 from DBC
      gas_pressed = (gas_pedal_raw > 5); // > 0.05 threshold
      brake_pressed = (brake_pedal_raw > 5); // > 0.05 threshold
    }

    // Monitor vehicle speed (if available in bus 0)
    if (addr == 0x220) {
      // Placeholder - need to map actual vehicle speed signal
      uint32_t speed_raw = GET_BYTES(msg, 0, 2) & 0xFFFFU;
      vehicle_moving = (speed_raw > 100); // Placeholder threshold
    }

    // Monitor cruise control buttons (PCM_BUTTONS message)
    if (addr == 0x3B0) {
      // Based on DBC: SET_BTN, RES_BTN, ACC_ON_BTN, LKAS_ON_BTN
      int acc_on_btn = GET_BIT(msg, 19);
      
      // Basic cruise control state management
      // This is a simplified implementation - may need refinement
      if (acc_on_btn) {
        acc_main_on = true;
      }
    }

    // Monitor steering angle from STEERING_MODULE_ADAS
    if (addr == 0x1E2) {
      // STEER_ANGLE from bit 24, 16 bits, scale 0.1, signed
      int steer_angle_raw = GET_BYTES(msg, 3, 2) & 0xFFFFU;
      int steer_angle = to_signed(steer_angle_raw, 16) / 10; // Scale by 0.1
      
      // Update steering angle sample
      update_sample(&angle_meas, steer_angle);
    }
  }
}

static bool byd_tx_hook(const CANPacket_t *msg) {
  const TorqueSteeringLimits BYD_STEERING_LIMITS = BYD_LIMITS(300, 3, 7);
  
  bool tx = true;
  int addr = GET_ADDR(msg);

  // STEERING_MODULE_ADAS: safety check for steering commands
  if (addr == 0x1E2) {
    // Extract STEER_ANGLE command (bit 24, 16 bits, signed, scale 0.1)
    int desired_angle_raw = GET_BYTES(msg, 3, 2) & 0xFFFFU;
    int desired_angle = to_signed(desired_angle_raw, 16);
    
    // Check if STEER_REQ is active (bit 21 from DBC)
    bool steer_req = GET_BIT(msg, 21);
    
    // Apply basic steering safety checks
    // Note: Using torque limits as placeholder - should implement angle limits
    if (steer_torque_cmd_checks(desired_angle / 10, steer_req, BYD_STEERING_LIMITS)) {
      tx = false;
    }
  }

  // ACC_CMD: safety check for longitudinal commands  
  if (addr == 0x32E) {
    // This would contain acceleration/deceleration commands
    // Implementation depends on actual message structure - need real vehicle data
    // For now, allowing all messages through with basic checks
    
    // Placeholder: Check for valid longitudinal command ranges
    // Real implementation would extract actual accel values and check against BYD_LONG_LIMITS
  }

  // Block diagnostic messages except tester present
  if ((addr == 0x7E0) || (addr == 0x7E8)) {
    // Only allow UDS tester present message
    if ((GET_BYTES(msg, 0, 4) != 0x00803E02U) || (GET_BYTES(msg, 4, 4) != 0x0U)) {
      tx = false;
    }
  }

  return tx;
}

static safety_config byd_init(uint16_t param __attribute__((unused))) {
  static RxCheck byd_rx_checks[] = {
    BYD_RX_CHECKS
  };

  // BYD specific initialization
  // param could be used for different BYD models (ATTO3, etc.)
  
  safety_config ret = BUILD_SAFETY_CFG(byd_rx_checks, BYD_TX_MSGS_LIST);
  return ret;
}

const safety_hooks byd_hooks = {
  .init = byd_init,
  .rx = byd_rx_hook,
  .tx = byd_tx_hook,
  .get_counter = byd_get_counter,
  .get_checksum = byd_get_checksum,
  .compute_checksum = byd_compute_checksum,
};
