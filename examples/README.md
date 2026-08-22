# Examples

No separate example scripts exist yet beyond the built-in CLI modes in
`pc/haptic_engine.py`, which serve as the working examples:

```
# See the encoding algorithm respond to a synthetic magnitude sweep,
# no eFlesh/sensor hardware required (Haptic ESP32 still needed):
python ../pc/haptic_engine.py --simulate --haptic-port COM8

# Same sweep, printed directly from the algorithm module, no serial at all:
python ../pc/haptic_algorithm.py --simulate
python ../pc/haptic_algorithm.py --example
```

`haptic_algorithm.py --example` prints a worked Bx/By/Bz -> motor-command
table showing baseline subtraction through to the final `M,...` output —
useful as a quick reference for the data shapes at each pipeline stage.
