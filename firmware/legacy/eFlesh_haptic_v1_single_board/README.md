# Legacy: single-board eFlesh prototype (superseded)

`eFlesh_haptic_v1.ino` was the **first** prototype: one ESP32-S3 reading
the MLX90393 *and* driving all six motors directly, with the entire
tactile-processing/haptic-encoding pipeline running on-device.

**This has been superseded by the current two-ESP32 + PC architecture**
(`../../sensor/eFlesh_sensor_v1/` + `../../haptic-controller/haptic_controller_v1/`
+ `../../../pc/`), which splits sensing, processing, and actuation across
separate boards with the PC as the processing layer. See
`../../../docs/architecture.md` for why.

Kept here for reference (e.g. if fully-embedded, PC-free operation is
revisited later — see "V1 vs future" in `../../../docs/haptic-algorithm.md`).
It is **not** part of the current pipeline and is not maintained in sync
with `pc/haptic_algorithm.py`'s tuning.
