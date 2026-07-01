# Assetto Corsa FIA DRS Rules

A Custom Shaders Patch (CSP) Lua app that enforces the real-Formula-1 **1-second DRS rule** on
both AI and the player, using the **FIA timing-loop model**. Stock AC AI (and many mod cars)
open DRS in every activation zone regardless of the gap to the car ahead; this app gates DRS on
the actual gap, the way a real race does.

Built and tested with the VRC Formula Alpha 2025 car for a private league. **League-safe:**
read-only with respect to car and track files (no checksum impact).

## The model

1. **Detection point ≠ zone start.** Each DRS zone has a *detection* spline (upstream) and a
   `[start … end]` activation window (downstream). The gap is sampled **once**, the instant a
   car crosses the detection line, and latched for the zone(s) that detection feeds.
2. **Gap = FIA timing-loop method** — the time between the car ahead crossing the detection
   point and this car crossing the *same* point (seconds). Speed-independent and accurate,
   unlike a distance/speed approximation.
3. **Authorization only.** The app decides one thing: does this car have DRS in this zone
   (gap ≤ 1.0 s)? The open/close dynamics inside an authorized zone stay with the car/native code.
4. **Cooperative suppression.** To deny DRS it makes DRS *unavailable* via the CSP physics API
   (`physics.allowCarDRS`), so the car's own controller closes the wing itself — no tug-of-war.
   It never force-opens DRS, and never touches authorized cars or cars outside a window.

## Options

Configurable in the app window: apply to **AI / Player / All**, gap threshold (default 1.0 s),
first DRS lap, race-only, lapped-car grant (FIA default: on), and an optional
"leader never gets DRS" league flavour. Optional gated file logging for debugging.

## Requirements

- Custom Shaders Patch with Lua apps enabled.
- The track's `surfaces.ini` must have `[_SCRIPTING_PHYSICS] ALLOW_APPS=1` for the physics API
  to be available.
- A track with a `drs_zones.ini` (detection + activation splines).

## Install

Copy this folder into `…/assettocorsa/apps/lua/vrc_fa25_drs_rules/`, enable it in CSP, and open
its window on track.

## Notes

Contains only my own original Lua. No mod car/track assets or third-party code are included.
Not affiliated with or endorsed by the VRC Modding Team.

## License

MIT — see [LICENSE](LICENSE).
