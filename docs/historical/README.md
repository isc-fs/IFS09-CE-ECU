# Historical documents — DO NOT TRUST THESE

Everything in this directory describes the **pre-rewrite C firmware**
(`control.c`, `can.c`, `app_state.c`) that **no longer exists**. They are kept
only because they occasionally explain *why* something was done, and because
deleting them would lose that.

**None of them describe the code on `dev`.** They were moved here out of `docs/`
because a footnote in CLAUDE.md was not enough: the filenames
(`ECU_ACU_MENSAJES.md`, `CAN_IDS_VARIABLES.md`) look authoritative when you find
them by grep, and they are not.

For anything current:

| you want | read |
|---|---|
| the CAN contract | `Core/Inc/can/messages/*.def` → [`../dbc/ecu.dbc`](../dbc/ecu.dbc) |
| the control logic | ``../../Core/Src/app/control.cpp`` |
| torque limiting | [`../derating.md`](../derating.md) |
| the dashboard contract | [`../CAN3_MAP.md`](../CAN3_MAP.md) |
| the radio contract | [`../RADIO_SNAPSHOT_MAP.md`](../RADIO_SNAPSHOT_MAP.md) |
| flashing / recovery | [`../flashing.md`](../flashing.md) |
| on-car bring-up | [`../commissioning.md`](../commissioning.md) |

If you find yourself citing a file in this directory to justify a change, stop —
you are almost certainly reading about firmware that was deleted.
