## How much difference does the Quadratic Damping make?
Other libraries (e.g. Roadrunner, Pedro Path) rely on gentle PID + coasting:

- Coasting (~`-40 in/s²`) needs ~`45in` to stop from `60 in/s`

With quadratic damping + back-EMF braking:

- ~`10 in` to stop from `60 in/s` → nearly **5× faster deceleration**
- Leaves more time for acceleration before braking

### Pros
- Faster - more time accelerating, less time decelerating
- More accurate - higher proportional constant reduces steady-state error
- More responsive - reacts quickly without oscillation due to quadratic damping

### Trade-offs
- Higher power usage - requires more voltage, applying reverse power to brake faster
- Sharper stops - can feel less smooth (minor shaking at final stop)

