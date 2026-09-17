/* SPDX-License-Identifier: proprietary */
/*
 * The ECU CAN message registry. can_codecs.hpp #includes THIS file once per
 * expansion pass (struct / encode / decode / descriptor / overlap-guard), so
 * adding a frame to the DSL is a single line here plus its .def.
 *
 * Scope: every frame the ECU participates in -- the frames it OWNS (TX) and
 * the frames it CONSUMES (RX) -- so the generated DBC documents the whole
 * contract and the encoders/decoders derive from one source. The `Sender`
 * field in each CAN_MSG tags ownership (VCU = this ECU, AMS, Pit_Tool, EMC).
 */

/* ---- ECU-owned TX ------------------------------------------------------- */
#include "vcu_heartbeat.def"        /* 0x100  DC-bus heartbeat -> ACU         */

/* uDV autonomous contract (#17), ACU bus; 0x500-0x502 reserved AS datalogger */
#include "vcu_ts_active.def"        /* 0x504  TS-active flag -> uDV           */
#include "vcu_brake_over_limit.def" /* 0x505  brake >= hard-braking limit     */
#include "vcu_motor_rpm.def"        /* 0x506  inverter rpm relay (SLAM)       */
#include "vcu_r2d_confirm.def"      /* 0x511  DV ready-to-drive confirm       */

/* GPS (MTK3339 on USART10). Ungated on the ACU bus: uDV localisation feed,
 * visible to the pit tool / MingoCAN for free (same bus + generated DBC). */
#include "vcu_gps_position.def"     /* 0x508  lat/lon, deg*1e7 s32 LE         */
#include "vcu_gps_status.def"       /* 0x509  speed km/h, course, sats, fix   */

/* pit-diag stream (gated by 0x7E0) */
#include "pit_diag_ack.def"         /* 0x7E1                                  */
#include "pit_cal_status.def"       /* 0x7E3  calibration session status      */
#include "pit_cal_apps.def"         /* 0x7E4  APPS endpoints read-back        */
#include "pit_cal_brake.def"        /* 0x7E5  brake values read-back          */
#include "pit_diag_status.def"      /* 0x700                                  */
#include "pit_diag_pedals.def"      /* 0x701                                  */
#include "pit_diag_brake.def"       /* 0x705                                  */
#include "pit_diag_inverter.def"    /* 0x702                                  */
#include "pit_diag_inverter_temps.def" /* 0x706  inverter temperatures        */
#include "pit_diag_inv_faults.def"  /* 0x708  inverter L1/L2 fault layers     */
#include "pit_diag_fwinfo.def"      /* 0x703                                  */
#include "pit_diag_health.def"      /* 0x704                                  */
#include "pit_diag_dv.def"          /* 0x707  DV drive-mode + autonomy (#109)  */
#include "pit_diag_cell.def"        /* 0x709  low-cell derate estimator        */
#include "pit_diag_pack_temp.def"   /* 0x70A  accumulator thermal cap          */
#include "pit_diag_inv_foc.def"     /* 0x70B  inverter FOC currents + modulus  */
#include "pit_diag_inv_torque.def"  /* 0x70C  ask / ceiling / delivered torque */
#include "pit_diag_power.def"       /* 0x70D  shaft vs AC vs DC -> efficiency  */

/* ---- RX consumed (sender owns the layout) ------------------------------- */
#include "acu_ok_precharge.def"     /* 0x020  AMS precharge-OK                */
#include "acu_discharge_interlock.def" /* 0x021 AMS fsm_in_start + tsms       */
#include "acu_v_cell_min.def"       /* 0x12C  AMS min cell voltage            */
#include "acu_vmin_module_a.def"    /* 0x131  AMS per-module vmin 0..2        */
#include "acu_vmin_module_b.def"    /* 0x132  AMS per-module vmin 3..4        */
#include "acu_vmax_module_a.def"    /* 0x133  AMS per-module vmax 0..2        */
#include "acu_vmax_module_b.def"    /* 0x134  AMS per-module vmax 3..4        */
#include "acu_currents.def"         /* 0x135  AMS accu/dcdc currents          */
#include "acu_tmax_module_a.def"    /* 0x136  AMS per-module tmax 0..2        */
#include "acu_tmax_module_b.def"    /* 0x137  AMS per-module tmax 3..4 + dcdc */
#include "ams_status.def"           /* 0x4A0  AMS FSM state / ams_ok          */
#include "udv_torque_cmd.def"       /* 0x507  uDV torque command (s32 LE, %)  */
#include "udv_as_status.def"        /* 0x50A  uDV autonomous-system state     */
#include "udv_r2d_request.def"      /* 0x510  uDV DV ready-to-drive request   */
#include "bl_boot_trigger.def"      /* 0x002  jump-to-bootloader magic        */
#include "pit_diag_cmd.def"         /* 0x7E0  pit-diag enable/disable magic   */
#include "pit_cal_cmd.def"          /* 0x7E2  operator calibration command    */

/*
 * TODO(inverter): the NX/EMC inverter frames (RX 0x461 state / 0x463 speed /
 * 0x466 DC-bus; TX 0x360 setpoint-1 / 0x362 setpoint-3) are AUTOSAR E2E
 * Profile 1 protected (CRC8 in byte 0 + 4-bit rolling counter). They land in
 * a follow-up once the E2E question is settled (does the inverter ENFORCE E2E
 * on RX? -- the old VCU sent plain zeros there). The E2E CRC/counter is
 * computed in the inverter adapter before encode, per the raw-struct/adapter
 * convention.
 */
