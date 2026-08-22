# SIH demonstration procedure

Target duration: 3-5 minutes. Keep the story technically honest — every
claim below matches what the current prototype actually does.

## Before the demo

- Both ESP32-S3 boards flashed and wired per `docs/hardware.md`.
- `calibration.json` present and tuned for the demo unit's eFlesh patch
  (`docs/calibration.md`) — do this well before the demo, not live.
- `python haptic_engine.py --sensor-port COMx --haptic-port COMy` running,
  dashboard visible on a screen the audience can see.
- eFlesh untouched during the startup baseline calibration message.

## Script

**1. Introduce the prosthetic.**
Show the 3D-printed robotic hand with the eFlesh patch mounted. State
plainly: this demo validates the sensory-feedback pathway — tactile
sensing on the hand, processed in real time, translated into vibration
the user can feel. It does not yet include hand actuation control or
EEG/EMG intent sensing.

**2. Show the e-Flesh.**
Point out the patch and the single MLX90393 magnetometer underneath it.
Mention explicitly: this prototype uses one eFlesh patch and one
magnetometer, by design, to get the core sensing-to-feedback loop
working reliably before scaling to a multi-sensor array.

**3. Demonstrate no contact.**
With the hand untouched, show the dashboard: `STATE: NO_CONTACT`,
intensity bar at 0, all six motors off. Let the audience confirm no
vibration.

**4. Apply light contact.**
Lightly touch/press the eFlesh. Show the dashboard reacting: `M`/`dx,dy,dz`
change, `STATE: CONTACT`, intensity bar rises off zero. One or two motors
begin a slow pulse — hand the unit to someone so they can feel it.

**5. Increase force/deformation.**
Press harder. Narrate what's changing: PWM amplitude rising, pulse rate
speeding up, more motors joining in (recruitment). This is the point to
say explicitly: intensity is relative, calibrated to this patch — not a
Newton reading.

**6. Show dashboard response.**
Call out the live terminal dashboard fields: sensor rate, Bx/By/Bz,
filtered magnitude, intensity bar, motor states, pulse pattern,
connection status for both ESP32 links. (There is currently no dedicated
graphical GUI for this pipeline — the terminal dashboard is the
real-time visualization; see `gui/README.md`.)

**7. Show motor recruitment.**
Explicitly point out that motor count increases with intensity (not just
amplitude) — light touch = 1-2 motors, strong = all six. This is a
deliberate design choice for perceptual distinguishability, not an
accident of wiring.

**8. Demonstrate release.**
Release contact. Dashboard returns to `NO_CONTACT`, motors stop. If the
release-pulse feature is enabled, mention the brief confirmation buzz on
release.

**9. Explain the Sentrix/FSR glove as the next calibration layer.**
State the honest next step: pairing this magnetic signal with a
ground-truth force glove (Sentrix/FSR) to build a calibrated force
model, rather than relying on relative thresholds. This is planned, not
built.

**10. Explain EEG as future intent-input integration.**
State clearly: EEG/sEMG-based intent control is a separate, future
integration, not part of this demo. This demo is the **afferent
(feedback) half** of the loop — sensing what the hand touches and
relaying it to the user. The efferent (control) half is future work.

## Honesty checklist for whoever presents

- Don't call `M`/`N`/`S` "force" or "Newtons."
- Don't claim slip/shear detection works — `SLIDING_OR_SHEAR` is a
  logged diagnostic label, not a validated feature.
- Don't claim multi-sensor eFlesh, EEG, or Sentrix integration exist yet.
- Don't cite the original eFlesh paper's accuracy numbers as this
  prototype's results.
