# firmware-flasher

Over-IR firmware updates for the Sensor Watch Pro. The host side lives in
`sensor-watch-ir-tools` (`bin/firmware_flasher.py`); this directory is the
watch side, minus the Movement face.

## Pieces

| file | runs from | role |
|---|---|---|
| `../watch-faces/settings/firmware_flasher_face.c` | flash | Movement face: menus, link TEST stage, ENTER validation, overlay loader, arming hand-off |
| `firmware_flasher_core.c` | RAM overlay | backend-agnostic core: NVM/DSU/SERCOM primitives, frame parser, ACK path, pull byte source, dispatch, `flasher_run` |
| `firmware_flasher_detools.c` | RAM overlay | detools crle patch decoder backend |
| `firmware_flasher_ultrapatch.c` | RAM overlay | UltraPatch decoder backend (the `ultrapatch` submodule's `patch_apply.h`) |
| `firmware_flasher_core.h` | — | cross-TU contract: geometry, wire constants, backend interface |
| `flasher-overlay.ld` | — | overlay placement fragment, appended to the chip script by `flasher-rules.mk` |
| `flasher.mk` / `flasher-rules.mk` | — | all build glue (backend selection, overlay capture, linker wrapper); the project Makefile includes one before gossamer's rules.mk, the other after |

Exactly one backend links per build: UltraPatch by default,
`FIRMWARE_FLASHER_ULTRAPATCH=0` for detools. The host tool's `--patch-format`
must match the watch build; a mismatch is refused before anything is written.
UltraPatch's submodule pin is the wire contract: the host CLI and this decoder
must build from the same headers.

## Why the odd build steps

The flasher erases the flash it executes from, so the core + backend must run
from RAM. They are captured **whole** (objcopy renames every allocated section
to `.flovl.*`; no per-function attributes to forget) into a load-on-demand
overlay near the top of RAM: zero boot-time RAM cost, copied in and
byte-verified by the face at first-block time — the point of no return. A
canary past the overlay guards against the session stack digging into the
code. RAM TUs may not call anything outside themselves (libc/libgcc live in
flash); `flasher-rules.mk`'s stamp rules fail the build if a banned symbol
appears.

Row writes retry a few times on NVMCTRL errors/timeouts and can rest between
bursts (`FLASHER_BURST_WRITE_COOLDOWN`, currently 0) to duty-cycle the NVM
current on the coin cell; a row that keeps failing parks the session,
recoverable over IR with a block-0 full-flash takeover.

## Protocol and testing

The stop-and-wait wire protocol is documented at the top of the face; the
decoders pull patch bytes through `patch_pull_byte` (contract identical to
UltraPatch's `PatchPull`). Heavy patch frames can legitimately take seconds
(hundreds of pages from a few input bytes) — the host's retransmissions and
the dup-re-ACK rule make that safe.

Any non-ARM compile of the RAM TUs is a hosted test build (keyed on the
compiler's `__arm__`): `sensor-watch-ir-tools/flasher-sim` links them against
fake link/flash layers and drives full sessions — including takeover, corrupt
patches, and write-failure parking — through the real `flasher_run`.
