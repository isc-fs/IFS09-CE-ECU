# Integration Guide: Electronic Differential (Torque Vectoring) for IFS-09

This document explains the software and hardware integration of the Rear-Wheel Drive (RWD) Electronic Differential subsystem into the `IFS09-CE-ECU` firmware architecture.

---

## 1. Safety & Formula Student Rules Compliance

### 1.1 Strict APPS Invariance (FS Rules T.11)
Formula Student rules mandate that accelerator pedal plausibility and emergency cuts must remain strictly authoritative:
1. **Plausibility & Disagreement (T.11.8.9):**
   - The dual APPS checks (`apps_pct`, agreement check, 100 ms persistence window) computed in `ecu::Controller::step()` must remain **completely untouched**.
   - If an APPS disagreement or brake-implausibility condition is detected, the base driver torque demand $T_{total}$ is set to $0\text{ Nm}$.
2. **Net Torque Preservation Invariant:**
   - The electronic differential takes $T_{total}$ as an input ceiling.
   - At all times:
     $$T_{left} + T_{right} \le T_{total}$$
   - The torque vectoring bias ($\Delta T$) transfers torque from inside wheel to outside wheel during a corner, but **never commands additional net forward torque beyond what the driver physically demanded**.
3. **Power Envelope (EV 2.2.1):**
   - The tractive power envelope ($80\text{ kW}$ ceiling) continues to cap the driver demand before or inside the differential allocator.

---

## 2. System Architecture & Task Interfacing

```
FDCAN RX ISR
     │
     ▼
can_rx_queue
     │
     ▼
CanRxTask (AboveNormal Priority)
     │
     ├─► Bosch LWS (0x0C0) ──► electronic_diff_can_process_rx() ──► VehicleService / DiffState
     ├─► Xsens IMU (0x201) ──► electronic_diff_can_process_rx() ──► VehicleService / DiffState
     └─► DTI FB (0x401/402)──► electronic_diff_can_process_rx() ──► VehicleService / DiffState

ControlTask (10 ms Realtime Priority)
     │
     ├─► 1. Read IO: Pedals (APPS1, APPS2), Brake
     ├─► 2. Step Core FSM: ecu::Controller::step() -> validated T_total
     ├─► 3. Check CAN Timeouts: electronic_diff_can_check_timeouts()
     ├─► 4. Step Diff Module: electronic_diff_step() -> T_left, T_right, I_left, I_right
     ├─► 5. Package Inverter CAN:
     │        - DTI Packet 0x01: Set AC Current (Left & Right)
     │        - DTI Packet 0x0C: Drive Enable
     └─► 6. Post to can_tx_queue -> CanTxTask -> FDCAN1
```

---

## 3. Step-by-Step Code Integration

### Step 3.1: Register Sensors in `CanRxTask`
In `Core/Src/app/can_rx_task.cpp`, route the shared vehicle CAN frames to `electronic_diff_can_process_rx()`:

```cpp
// Inside ecu_can_rx_task_run() frame dispatch loop:
bool processed = electronic_diff_can_process_rx(
    &g_diff_config,
    f.id,
    (f.flags & CAN_FLAG_EXT_ID) != 0,
    f.data,
    f.dlc,
    osKernelGetTickCount(),
    &g_lws_state,
    &g_imu_state,
    &g_dti_left_state,
    &g_dti_right_state
);
```

### Step 3.2: Periodic Execution in `ControlTask` (10 ms)
In `Core/Src/app/control_task.cpp`, execute the electronic differential step directly after `ctrl.step()` derives the validated driver torque:

```cpp
// 1. Run timeout guards
electronic_diff_can_check_timeouts(
    &g_diff_config,
    now,
    &g_lws_state,
    &g_imu_state,
    &g_dti_left_state,
    &g_dti_right_state
);

// 2. Prepare differential inputs
DiffInputs diff_in{};
// Base driver torque in Nm (e.g. torque_pct mapped to total shaft Nm)
diff_in.total_torque_demand_nm   = (float)out.torque_nm;
diff_in.steering_wheel_angle_rad = g_lws_state.steering_angle_rad;
diff_in.steering_angle_valid     = g_lws_state.valid;
// Estimate v_x from rear wheel ERPM average
diff_in.left_wheel_speed_rads    = g_dti_left_state.wheel_speed_rads;
diff_in.right_wheel_speed_rads   = g_dti_right_state.wheel_speed_rads;
diff_in.wheel_speeds_valid       = g_dti_left_state.valid && g_dti_right_state.valid;
diff_in.vehicle_speed_mps        = (diff_in.wheel_speeds_valid)
    ? (0.5f * (diff_in.left_wheel_speed_rads + diff_in.right_wheel_speed_rads) * g_diff_config.vehicle.wheel_radius_m)
    : 0.0f;
diff_in.measured_yaw_rate_rads   = g_imu_state.yaw_rate_rads;
diff_in.lateral_accel_mps2       = g_imu_state.lateral_accel_mps2;
diff_in.imu_valid                = g_imu_state.valid;
diff_in.max_motor_torque_limit_nm= g_diff_config.vehicle.motor_max_torque_nm;
diff_in.dt_s                     = 0.010f;
diff_in.now_ms                   = now;

// 3. Execute Electronic Differential algorithm
DiffOutputs diff_out{};
if (out.state == CtrlState::Active) {
    electronic_diff_step(&g_diff_state, &g_diff_config, &diff_in, &diff_out);
} else {
    electronic_diff_init(&g_diff_state);
}

// 4. Transmit DTI Inverter commands via can_tx_post()
uint32_t can_id = 0;
uint8_t dlc = 0;
uint8_t data[8];

// Command Left Inverter
electronic_diff_can_build_dti_set_current(
    &g_diff_config,
    g_diff_config.can.dti_left_node_id,
    diff_out.current_left_apk,
    &can_id, data, &dlc
);
CanFrame f_left{ static_cast<uint8_t>(CanBus::Inv), can_id, dlc };
memcpy(f_left.data, data, dlc);
can_tx_post(f_left);

// Command Right Inverter
electronic_diff_can_build_dti_set_current(
    &g_diff_config,
    g_diff_config.can.dti_right_node_id,
    diff_out.current_right_apk,
    &can_id, data, &dlc
);
CanFrame f_right{ static_cast<uint8_t>(CanBus::Inv), can_id, dlc };
memcpy(f_right.data, data, dlc);
can_tx_post(f_right);
```

---

## 4. Mode Switching & Failsafe Logic

| Condition | System Action | Operating Mode |
|-----------|---------------|----------------|
| **Normal Operation (IMU + Steering OK)** | Full closed-loop yaw rate feedback + load transfer | `DIFF_MODE_PARTY` |
| **IMU Lost / Stale (> 50 ms)** | Instant glitch-free rate-limited fallback | `DIFF_MODE_MINGO` |
| **IMU Recovers** | Debounces for 150 ms continuous healthy frames before re-promoting | `DIFF_MODE_MINGO` $\to$ `DIFF_MODE_PARTY` |
| **Steering Sensor Lost (> 100 ms)** | Symmetric 50/50 fallback distribution ($T_{base} = T_{total} / 2$) | `DIFF_MODE_PASSIVE` |
| **High Wheel Slip (> 15% delta)** | Virtual LSD trims torque on spinning wheel | Active in `MINGO` |
| **Spin Detected ($|\beta| > 8.5^\circ$)** | Emergency cut: $T_{total} \times 0.3$ + counter-yaw moment | Active in `PARTY` |
| **Inverter Fatal Fault (DRV / Overcurrent)**| Drive Enable commanded to 0, torque zeroed | Emergency Inhibit |

---

## 5. Trackside Tuning Guide (`differential_config.h`)

All tuning constants are grouped in `Core/Inc/app/differential_config.h` and initialized in `Core/Src/app/differential_config.c`:

1. **Understeer Tuning (Corner Entry & Mid-Corner):**
   - Increase `pid.kp` (default `18.0 Nm/(rad/s)`) if the car pushes wide during corner entry.
   - If the rear end feels loose on corner entry, reduce `pid.kp` or increase `gain_table.deadband_angle_rad`.
2. **High-Speed Stability:**
   - Verify `gain_table.k_speed_bp[5]` and `k_speed_bp[6]` (> 62 km/h). The torque vectoring gain is purposefully tapered down to `0.15` at high speeds to prevent violent yaw kicks on straightline bumps.
3. **Corner Exit Traction:**
   - Adjust `limits.lsd_slip_threshold` (default `0.15`) and `limits.lsd_k_slip` (default `1.50`). Lowering the threshold causes the Virtual LSD to intervene earlier when accelerating across kerbs.
