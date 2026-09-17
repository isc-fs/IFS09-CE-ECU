// SPDX-License-Identifier: proprietary
//
// Emits the `bl_fwinfo_t` record the isc-fs/stm32-can-bootloader requires at a
// fixed flash offset to consider the application valid. WITHOUT this record the
// bootloader rejects JUMP with `NO_VALID_APP` even after FLASH_WRITE +
// FLASH_VERIFY succeed -- i.e. the app silently refuses to boot. This is one of
// the two pieces that make the ECU "flashable like the AMS".
//
// Layout is taken verbatim from isc-fs/stm32-can-bootloader/Core/Inc/bl_fwinfo.h
// (64 bytes, magic 0xF14F1B00, record schema 1.0). We can't #include that header
// (the BL repo isn't a submodule of this tree), so the struct is replicated here
// and a static_assert guards the 64-byte size. If the BL bumps the magic or the
// layout, follow it here in lockstep.
//
// The record is pinned at BL_APP_BASE + 0x400 (= 0x08020400) by the
// `.firmware_info` section in STM32H733ZGTX_FLASH.ld.
//
// Version / git_hash / build_timestamp come from CMake at configure time
// (ECU_FW_VERSION_*, ECU_GIT_HASH_*, ECU_BUILD_TIMESTAMP). The fallbacks below
// keep this TU compilable in a host build with a zero-filled id block.

#include "app/ecu_config.hpp"

#include <cstdint>

#ifndef ECU_FW_VERSION_MAJOR
#  define ECU_FW_VERSION_MAJOR 0
#endif
#ifndef ECU_FW_VERSION_MINOR
#  define ECU_FW_VERSION_MINOR 0
#endif
#ifndef ECU_FW_VERSION_PATCH
#  define ECU_FW_VERSION_PATCH 0
#endif
#ifndef ECU_GIT_HASH_0
#  define ECU_GIT_HASH_0 0
#endif
#ifndef ECU_GIT_HASH_1
#  define ECU_GIT_HASH_1 0
#endif
#ifndef ECU_GIT_HASH_2
#  define ECU_GIT_HASH_2 0
#endif
#ifndef ECU_GIT_HASH_3
#  define ECU_GIT_HASH_3 0
#endif
#ifndef ECU_BUILD_TIMESTAMP
#  define ECU_BUILD_TIMESTAMP 0
#endif

extern "C" {

typedef struct {
    uint32_t magic;
    uint32_t record_version;
    uint32_t fw_version_major;
    uint32_t fw_version_minor;
    uint32_t fw_version_patch;
    uint32_t mcu_id;
    uint8_t  git_hash[8];
    uint64_t build_timestamp;
    char     product_name[16];
    uint32_t reserved[2];
} bl_fwinfo_t;

static_assert(sizeof(bl_fwinfo_t) == 64,
              "bl_fwinfo_t must be exactly 64 bytes (stm32-can-bootloader contract)");

const bl_fwinfo_t __firmware_info
    __attribute__((section(".firmware_info"), used)) = {
    .magic            = 0xF14F1B00U,        // BL_FWINFO_MAGIC
    .record_version   = 0x00010000U,        // record schema 1.0
    .fw_version_major = ECU_FW_VERSION_MAJOR,
    .fw_version_minor = ECU_FW_VERSION_MINOR,
    .fw_version_patch = ECU_FW_VERSION_PATCH,
    .mcu_id           = 0x00000450U,        // STM32H7x3
    // First 4 bytes of the 8-char git short hash (CMake); trailing 4 stay zero.
    .git_hash         = { ECU_GIT_HASH_0, ECU_GIT_HASH_1,
                          ECU_GIT_HASH_2, ECU_GIT_HASH_3,
                          0, 0, 0, 0 },
    .build_timestamp  = static_cast<uint64_t>(ECU_BUILD_TIMESTAMP),
    .product_name     = "IFS08-CE-ECU",
    // reserved[0]: the ECU's BL node id. The flasher reads it and refuses to
    // flash if it doesn't match the BL it just discovered. reserved[1] free.
    .reserved         = { ecu::config::EcuNodeId, 0 },
};

// Pit-diag (0x703) accessors -- let CanTxTask surface firmware identity on the
// wire without re-declaring the bl_fwinfo_t shape elsewhere.
uint8_t        ecu_fw_version_major(void) { return static_cast<uint8_t>(__firmware_info.fw_version_major); }
uint8_t        ecu_fw_version_minor(void) { return static_cast<uint8_t>(__firmware_info.fw_version_minor); }
uint8_t        ecu_fw_version_patch(void) { return static_cast<uint8_t>(__firmware_info.fw_version_patch); }
uint8_t        ecu_bl_node_id(void)       { return static_cast<uint8_t>(__firmware_info.reserved[0]); }
const uint8_t* ecu_git_hash(void)         { return __firmware_info.git_hash; }

}  // extern "C"
