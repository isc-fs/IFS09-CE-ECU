// SPDX-License-Identifier: proprietary
//
// dbc_dump -- emit DBC BO_/SG_ rows straight from the code-first CAN DSL
// descriptors (ecu::ALL_MSGS). The committed docs/dbc/ecu.dbc IS this tool's
// output; the dbcinator bot (.github/workflows/dbc-bot.yml) regenerates it on
// every PR and force-pushes the result, so the DBC can never drift from the
// .def files the firmware encoders are generated from. One source of truth.
//
// C++ TU against the rewrite's C++ DSL (can_codecs.hpp) -- mirrors the AMS
// tools/dbc_dump.cpp (the ECU has no array-of-frames families, so that section
// is dropped). Defining ECU_CAN_DESCRIPTORS pulls in the pass-4 descriptor
// tables (ecu::ALL_MSGS of can_dsl::MsgDesc).
//
// Build + run (host):
//   c++ -std=c++17 -I Core/Inc tools/dbc_dump.cpp -o /tmp/dbc_dump && /tmp/dbc_dump

#define ECU_CAN_DESCRIPTORS
#include "can/can_codecs.hpp"

#include <cstdio>
#include <cstring>

int main()
{
    // DBC header boilerplate (cantools-parseable). Lean by design (no CM_ /
    // VAL_ tables); DBCinator re-enriches downstream.
    std::printf("VERSION \"\"\n\n");
    std::printf("NS_ :\n");
    static const char* ns[] = {
        "NS_DESC_", "CM_", "BA_DEF_", "BA_", "VAL_", "CAT_DEF_", "CAT_", "FILTER",
        "BA_DEF_DEF_", "EV_DATA_", "ENVVAR_DATA_", "SGTYPE_", "SGTYPE_VAL_",
        "BA_DEF_SGTYPE_", "BA_SGTYPE_", "SIG_TYPE_REF_", "VAL_TABLE_", "SIG_GROUP_",
        "SIG_VALTYPE_", "SIGTYPE_VALTYPE_", "BO_TX_BU_", "BA_DEF_REL_", "BA_REL_",
        "BA_DEF_DEF_REL_", "BU_SG_REL_", "BU_EV_REL_", "BU_BO_REL_", "SG_MUL_VAL_",
    };
    for (unsigned i = 0u; i < sizeof(ns) / sizeof(ns[0]); ++i) std::printf("\t%s\n", ns[i]);
    std::printf("\nBS_:\n\n");
    std::printf("BU_: AMS VCU ECU UDV Pit_Tool\n\n");

    // GenMsgCycleTime attribute declaration. cantools exposes this as
    // Message.cycle_time; DBCinator's bus_load.py reads it to size the fleet
    // bus budget. Default 0 means "no declared cadence".
    std::printf("BA_DEF_ BO_  \"GenMsgCycleTime\" INT 0 65535;\n");
    std::printf("BA_DEF_DEF_  \"GenMsgCycleTime\" 0;\n\n");

    for (unsigned mi = 0u; mi < ecu::ALL_MSGS_COUNT; ++mi) {
        const can_dsl::MsgDesc* m = &ecu::ALL_MSGS[mi];
        std::printf("BO_ %u %s: %u %s\n", m->id, m->name, m->dlc, m->sender);
        for (unsigned fi = 0u; fi < m->n_fields; ++fi) {
            const can_dsl::FieldDesc* f = &m->fields[fi];
            // The [min|max] range is required for cantools to parse the SG_
            // line; emit [0|0] ("unspecified"). Real ranges are display polish
            // DBCinator can enrich downstream.
            std::printf(" SG_ %s : %u|%u@%c%c (%g,%g) [0|0] \"%s\" Vector__XXX\n",
                        f->name, f->start_bit, f->len,
                        f->big_endian ? '0' : '1', f->is_signed ? '-' : '+',
                        f->factor, f->offset, f->unit);
        }
        std::printf("\n");
    }

    // Per-message GenMsgCycleTime attributes, emitted after the BO_ block.
    // Period 0 means one-shot / on-demand -- skipped (BA_DEF_DEF_ covers it).
    for (unsigned mi = 0u; mi < ecu::ALL_MSGS_COUNT; ++mi) {
        const can_dsl::MsgDesc* m = &ecu::ALL_MSGS[mi];
        if (m->period_ms > 0u) {
            std::printf("BA_ \"GenMsgCycleTime\" BO_ %u %u;\n", m->id, m->period_ms);
        }
    }

    // VAL_ tables (enum value names) from the DSL CAN_VAL rows. Contiguous rows
    // sharing the same (msg_id, signal) collapse into one VAL_ line -- so the pit
    // tool decodes e.g. fsm_state=5 as "Active", last_fault=243 as "BusFault".
    std::printf("\n");
    for (unsigned i = 0u; i < ecu::ALL_VALS_COUNT;) {
        const can_dsl::ValRow* v = &ecu::ALL_VALS[i];
        std::printf("VAL_ %u %s", v->msg_id, v->signal);
        unsigned j = i;
        while (j < ecu::ALL_VALS_COUNT &&
               ecu::ALL_VALS[j].msg_id == v->msg_id &&
               std::strcmp(ecu::ALL_VALS[j].signal, v->signal) == 0) {
            std::printf(" %u \"%s\"", ecu::ALL_VALS[j].value, ecu::ALL_VALS[j].name);
            ++j;
        }
        std::printf(" ;\n");
        i = j;
    }
    return 0;
}
