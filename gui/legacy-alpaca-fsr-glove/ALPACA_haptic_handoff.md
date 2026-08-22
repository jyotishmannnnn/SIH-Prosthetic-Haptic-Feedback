# ALPACA Haptic Feedback Demo — Full Context Handoff

Paste this whole document into a new chat to resume work with complete context.

---

## WHO / WHAT

**Team Mirai** — Startup India Hackathon, Problem Statement 52 (Smart Prosthetics).

**Product:** ALPACA (Aluminium + Polyamide Carbon) — a sub-$500 hybrid bionic hand.
AlSi10Mg metal skeleton at high-stress nodes (wrist interface, palm spine, thumb CMC,
MCP carriers, tendon anchors), PA-CF (carbon-fibre nylon) for phalanges, palm body,
tendon channels and replaceable modules. Steel pins, PTFE tendon guides, TPU skin.

**Full-system targets (design targets, NOT measured results):**
- ~300 g bare hand structure, ~₹10,000
- Full BOM ₹47,807 (~$499.5 at ~₹95.7/USD)
- ~1.22 kg complete system mass
- 8–12 N fingertip force; 5–8 kg whole-hand static structural target
- 5× Waveshare ST3215 servos, ESP32-C6, 8-ch ADS1299 front end, DRV2605L haptics

**Team:** Sharad Kumar Mishra (lead), Jyotishman Das, Abhishek Raj,
Abdur Raheem Imami, Sanchali Agare, Blesson T Reji.

---

## WHAT I'M BUILDING RIGHT NOW

A **proof-of-concept of the afferent (feedback) half only**. Pitch is imminent.

Sensing glove → fingertip force → vibrotactile band on forearm.
The EMG/control half is NOT in this demo. The 3D-printed hand exists but is not
yet instrumented — the glove stands in for its fingertip sensors.

### Existing, already working
- Sensing glove with 5 FSRs, each with a 10 kΩ pulldown resistor (voltage divider)
- **Seeed Studio XIAO ESP32-S3**, powered over USB-C
- A GUI that displays per-finger pressure as a live heatmap
- Force already converted to a **0–20 N** range in software, in a `float force[5]` array

### Being added (this work)
- 6 coin ERM vibration motors on a Myo-band-style forearm strap
- Force → discrete level → PWM + rhythm mapping
- Software current budgeting (no capacitors, no flyback diodes)

---

## HARDWARE

### Pin assignment (XIAO ESP32-S3)

| Pin | GPIO | Function |
|---|---|---|
| D0–D4 | — | 5× FSR inputs (ADC1) — already wired |
| D5 | 6 | Motor 0 — Thumb |
| D8 | 7 | Motor 1 — Index |
| D9 | 8 | Motor 2 — Middle |
| D10 | 9 | Motor 3 — Ring |
| D7 (RX) | 44 | Motor 4 — Little |
| D6 (TX) | 43 | Motor 5 — mirrors hardest-pressed finger |

Motor 5 is on TX deliberately: GPIO43 carries the ROM bootloader log and blips
for ~200 ms at reset. A blip on the mirror channel is unnoticeable; on a named
finger it would look like a sensor glitch.

### Motor driver — BC557 (PNP), high-side, one per motor

```
3.3V ── EMITTER
        BC557
        COLLECTOR ── MOTOR +
                     MOTOR − ── GND
GPIO ── 330Ω ── BASE
```

- **GPIO LOW = motor ON** (PNP). Code inverts duty internally.
- BC557 TO-92 pinout, flat face toward you, legs down = **C – B – E**
  (opposite of a 2N2222 — a common wiring mistake).
- Works because motor rail is 3.3 V and GPIO high is 3.3 V, so HIGH fully turns off.
  **This would break on a 5 V motor rail** — would need a second transistor.
- Base current ≈ (3.3 − 0.7)/330 ≈ 8 mA, well saturated for an ~80 mA motor.

### Deliberate decisions — do not re-litigate these

- **No flyback diodes.** At ~80 mA with a coin ERM's tiny inductance and BC557's
  ~45 V V_CEO headroom, the turn-off spike is a lifetime concern, not a
  will-it-work concern. Tested and working. Decision made.
- **No decoupling capacitor.** Brownout is handled in software via a
  concurrency budget (`MAX_ON`) instead of smoothing the transient.
- **No base pull-up resistors.** A floating PNP base leaks in microamps and won't
  spin a motor; `digitalWrite(HIGH)` before `ledcAttach` covers the boot window.
- **No aggregate/grip channel.** Motor 5 simply mirrors the peak finger.

---

## THE MAPPING

```
FSR + 10k divider → ADC (0-4095) → Force (0-20 N) → Level (0-4) → PWM → motor
      [working]        [working]      [working]        [new]      [new]
```

### Level ladder

| Level | Force | PWM | Pattern | Perceived as |
|---|---|---|---|---|
| 0 | 0–1 N | 0 | silent | nothing |
| 1 | 1–5 N | 110 | 60 ms on / 440 ms off | slow tick |
| 2 | 5–10 N | 160 | 70 ms on / 180 ms off | steady tap |
| 3 | 10–15 N | 210 | 80 ms on / 60 ms off | fast flutter |
| 4 | 15–20 N | 255 | continuous | solid buzz |

**Why PWM starts at 110, not near zero:** coin ERMs don't spin below roughly
40% duty on a 3.3 V rail. Anything lower is a dead zone.

**Why rhythm changes alongside amplitude:** untrained users reliably distinguish
only 3–4 vibration amplitudes. With amplitude alone, levels 2 and 3 feel identical
to a first-time wearer and the demo falls flat. Co-modulating rhythm makes the
levels obvious within ~2 seconds with no training.

**Why `MAX_ON = 3`:** six motors at ~80 mA is ~480 mA against a ~700 mA regulator,
and simultaneous startup surges push past it → board reboots. Capping concurrent
motors solves this in software. Bonus: six motors buzzing at once mechanically
fuse into one undifferentiated blur on the forearm (tactile masking), so muting
the weakest channels actually *improves* localisation.

---

## CODE

Add to the existing glove sketch. Call `motorSetup()` in `setup()` and
`updateMotors(force)` in `loop()`, where `force[5]` is the 0–20 N array already
feeding the heatmap. Nothing else in the sketch changes.

```cpp
// ================= ALPACA haptic band =================
const int MOTOR[6] = {D5, D8, D9, D10, D7, D6};
//                   thumb idx  mid  ring  little  mirror

const int PWM_LV[5] = {0, 110, 160, 210, 255};
const int ON_MS[5]  = {0,  60,  70,  80,   0};   // 0 at L4 = continuous
const int OFF_MS[5] = {0, 440, 180,  60,   0};

#define MAX_ON 3          // current budget: 3 x 80mA = 240mA

bool     isOn[6]  = {false};
uint32_t tMark[6] = {0};

int levelFromForce(float N) {
  if (N <  1) return 0;
  if (N <  5) return 1;
  if (N < 10) return 2;
  if (N < 15) return 3;
  return 4;
}

void drive(int m, int duty) {
  ledcWrite(MOTOR[m], 255 - duty);        // PNP inverted
}

void motorSetup() {
  for (int m = 0; m < 6; m++) {
    pinMode(MOTOR[m], OUTPUT);
    digitalWrite(MOTOR[m], HIGH);         // off before PWM attaches
    ledcAttach(MOTOR[m], 20000, 8);       // 20 kHz = inaudible switching
    drive(m, 0);
    tMark[m] = millis() + m * 20;         // stagger startup surge
  }
}

// call every loop with the existing 0-20 N array
void updateMotors(float forceN[5]) {
  uint32_t now = millis();

  float peak = 0;
  for (int i = 0; i < 5; i++) peak = max(peak, forceN[i]);

  int lv[6], want[6], n = 0;
  for (int m = 0; m < 6; m++) {
    lv[m] = levelFromForce(m < 5 ? forceN[m] : peak);

    if (lv[m] == 0)      { want[m] = 0; }
    else if (lv[m] == 4) { want[m] = 255; }
    else {
      int dur = isOn[m] ? ON_MS[lv[m]] : OFF_MS[lv[m]];
      if (now - tMark[m] >= dur) { isOn[m] = !isOn[m]; tMark[m] = now; }
      want[m] = isOn[m] ? PWM_LV[lv[m]] : 0;
    }
    if (want[m] > 0) n++;
  }

  while (n > MAX_ON) {                    // mute weakest over budget
    int weakest = -1;
    for (int m = 0; m < 6; m++)
      if (want[m] > 0 && (weakest < 0 || lv[m] < lv[weakest])) weakest = m;
    want[weakest] = 0;
    n--;
  }

  for (int m = 0; m < 6; m++) drive(m, want[m]);
}
```

**Core version note:** `ledcAttach(pin, freq, res)` is Arduino-ESP32 core 3.x.
On core 2.x use `ledcSetup(ch, 20000, 8)` + `ledcAttachPin(pin, ch)` and write to
channel numbers instead of pins.

---

## EXECUTION PLAN (~2 hours)

**1. IDE setting (2 min).** Tools → **USB CDC On Boot → Enabled**.
Without this, `Serial` grabs UART0 and motors 4–5 buzz in time with telemetry.

**2. Solder the band (45 min).** 6× BC557 + 330 Ω, high-side.
Space motors **≥40 mm apart** — two-point discrimination on the forearm is
~35–40 mm, and below that nobody can tell which finger fired.
**Forearm, not bicep** (bicep discrimination is ~45+ mm, worse).

**3. Bench test, no glove (15 min).** Temporary loop stepping each motor 0→4.
Confirm: nothing spins at boot; each motor responds; all stop cleanly.
Meter across a motor at full should read **~3.1 V**.
If it reads 1.5–2.2 V and the transistor is warm → C and E are swapped.

**4. Wear it and tune (20 min).** Run one channel up the ladder repeatedly.
If L2 and L3 feel the same, widen the gap — drop `PWM_LV[2]` to ~140.
Must be done on skin; cannot be reasoned out.

**5. Connect the glove (20 min).** Press each finger; confirm the correct motor
fires and the heatmap agrees. If the heatmap jitters when motors run, twist the
motor wires and route them away from FSR wires.

**6. Rehearse (20 min).** Four grips: light touch one finger; hard press one
finger; precision pinch (thumb + index); full grasp.

---

## PITCH FRAMING

Hand the band to a judge, press fingertips yourself, let them feel it.

Say this unprompted:

> "The glove is standing in for the hand's fingertip sensors — same signal path,
> same encoding. We're demonstrating the afferent half of the loop; EMG control
> is the next integration step."

Disclosed limitation reads as engineering discipline. Discovered limitation reads
as concealment. Same fact, opposite outcome.

**Anticipated Q — why not scale voltage linearly with force?**
Untrained users resolve only 3–4 amplitude steps, so intensity is encoded in
rhythm as well as amplitude.

**Anticipated Q — why not run all six motors at once?**
Parallel actuation mechanically fuses into a single masked buzz and destroys
spatial localisation. The concurrency cap improves discrimination and doubles as
a current budget.

**Anticipated Q — does vibrotactile feedback actually help users?**
Honest answer: evidence is reasonably good for improved grasp-force modulation
and prosthesis embodiment; it does not restore anything like real touch.
Claim the former only.

---

## KNOWN OPEN ISSUES ON THE WIDER PROJECT

Flagged in earlier analysis, not blocking this demo, but likely to come up:

1. **EEG vs sEMG contradiction.** The one-liner says "EEG brain signals";
   the technical summary says "sEMG front end". Must be resolved to one story.
   Recommendation: sEMG as the control path (fast, proportional, reliable),
   EEG positioned as an optional research channel. Non-invasive EEG gives only
   2–4 classes with seconds of dwell time — it cannot drive dexterous control.
2. **1.22 kg system mass** vs ~450–600 g for commercial myoelectric hands.
   Device weight is a top driver of prosthesis abandonment. Mitigation: mount
   actuators on the forearm and quote distal mass separately.
3. **5 actuators** = one per digit, no independent thumb opposition. Avoid
   side-by-side comparison with 16-DOF and 21-DOF research hands.
4. **LPBF AlSi10Mg cost/lead time.** Small low-volume parts attract minimum-order
   charges; realistically ₹15,000–35,000 for a one-off, and 2–4 weeks.
   Be ready to justify LPBF over CNC 6061 (which is cheaper, faster, and has
   better fatigue behaviour unless the geometry is genuinely topology-optimised).
5. **No socket in the BOM.** Patient-specific socket fitting is often the most
   expensive, skill-intensive part. Either scope it out explicitly or budget it.
6. **"$500" is a quantity-one BOM, not a price.** Realistic retail is ₹1.5–2.5 lakh
   — still 5–10× below imported myoelectric hands, which is the stronger claim.
7. **Regulatory path unaddressed.** CDSCO device classification, IEC 60601 for
   skin-contact electrodes and battery safety. One slide showing awareness is
   disproportionately valuable at a government-backed hackathon.

**Reference worth citing:** CleverHand (github.com/Aightech/CleverHand) — open
modular EMG + vibrotactile HMI platform. Not usable directly (raw KiCad, needs
fabrication) but independently validates the same market gap between ~$40 hobby
EMG and ~$2,000 lab systems.

---

## HOW TO PICK UP

Reply with where you're stuck. Likely next steps:
- Debugging motors that don't spin / spin weakly (check C–E orientation first)
- Tuning the ladder after wearing it
- Integrating `updateMotors()` into the existing glove loop
- Adding a scripted fallback mode (hardcoded scenarios, no sensors) as demo insurance
- Pitch deck / slide structure
