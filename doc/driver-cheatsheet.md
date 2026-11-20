# BIOBUZZ driver cheat sheet

TeleOp **BIOBUZZ**, one PlayStation controller (gamepad 1). Keep this in sync with `BIOBUZZ.java`.

## Before the match

1. **Battery** charged (telemetry `Battery` should be above ~12.5 V at the start).
2. Put the robot on its start spot, **facing away from the drivers** (toward the HIVE).
3. Press **INIT** and check the `Turret` line: it should say about **0° from centre** with the turret pointing straight ahead.
   - First INIT after switching the robot on: **centre the turret by hand first**. That position becomes its zero and is kept for every later OpMode until the robot is switched off.
   - Turret centred but the line says something else (or warns)? **Hold Options** during INIT to re-zero it there.
4. Pick the alliance: **Circle = RED, Cross = BLUE**. Check the `Alliance` line.
5. Press **Start**.

## Controls

```
        L2 intake out                       R2 intake in
        L1 turret left                      R1 turret right
   Share: odometry / assists           Options: re-zero odometry
   D-pad up/down: which CELL is up     Triangle: auto-aim on/off
   Left stick: drive                   Square: shoot (hold)
   L3 (press): field-relative          Cross / Circle: transfer fwd / back
                                        Right stick X: turn
                                        R3 (press): reset "away from drivers"
```

| Control | What it does |
|---|---|
| **Left stick** | Drive and strafe |
| **Right stick X** | Turn |
| **R2 / L2** | Intake in / out |
| **Triangle** | Auto-aim **on/off** (starts OFF) |
| **R1 / L1** | Turn the turret right / left by hand (only while auto-aim is off) |
| **Square, auto-aim ON** | **Hold to shoot.** Balls feed only while the shot is READY (see below) |
| **Square, auto-aim OFF** | Hold to spin the flywheel at 2000 RPM (you feed with Cross) |
| **Cross / Circle** | Transfer forward / backward, by hand (e.g. to clear a jam) |
| **D-pad up / down** | The FAR / AUDIENCE CELL is tipped up (only for odometry aim) |
| **Options** | Robot is back on its start spot: re-zero odometry there (also forgets the remembered goal) |
| **Share, tap** | **Odometry off/on**, e.g. if a pod gets knocked loose |
| **Share, hold ~0.6 s** | **Driving assists off/on** |
| **L3** (press left stick) | **Field-relative** driving on/off: stick forward = away from the drivers |
| **R3** (press right stick) | "Away from the drivers" = the way the robot faces right now |

## Shooting

1. Press **Triangle**. The turret finds the goal and the flywheel spins up for the distance.
2. Wait for the **rumble**. It means the shot is READY.
3. **Hold Square.** Balls keep feeding as long as the shot stays READY, so you can hold it while driving past the HIVE.

The `Shot` line says READY or what it is waiting for:

| `Shot` says | Meaning / what to do |
|---|---|
| `goal not seen` | The camera doesn't see your goal and there's no recent remembered goal. Turn toward the HIVE. |
| `distance … outside shot table` | Too close or too far (table covers about 36–86 in). Move. |
| `aiming (x°)` | The turret is still turning onto the goal. Wait a moment. |
| `rpm x/y` | The flywheel isn't at speed yet. If it stays low, the battery is probably weak. |
| `goal behind the turret's blind spot` | The turret can't point there (cable limits). Turn the robot. |

- The turret reaches both sides (right ~+90°, left ~-90°). Directly behind the robot is a blind spot.
- When the camera loses the goal (too close, outside its view), the turret keeps aiming at the **remembered goal** using odometry (`Tracking Status: Odometry (remembered goal)`), for up to 15 s after the last sighting.
- While moving, the robot leads its shots automatically.

## Driving assists (on by default)

- **Straight zone:** push the stick *nearly* straight (within 8°) and the robot drives *exactly* straight, forward/back or sideways.
- **Heading hold:** let go of the turn stick and the robot keeps facing the same way.
- **Line hold:** in the straight zone the robot stays on the line it started on.
- **Field-relative (L3):** stick forward = away from the drivers, whichever way the robot faces. Handy when it's turned around. If "forward" feels wrong, point the robot away from the drivers and press **R3**.

## If something goes wrong

| Problem | Do this |
|---|---|
| Robot drives or aims strangely after a hit; a pod might be loose | **Tap Share** (odometry off). Auto-aim keeps working with the camera only. |
| Driving feels wrong / fights you | **Hold Share** (assists off). Plain driving, like before. |
| Field-relative "forward" is off | Point the robot away from the drivers, press **R3**. Or turn it off with **L3**. |
| Ball stuck in the transfer | Hold **Circle** briefly (transfer backward). |
| Turret aims at the wrong HIVE | Wrong alliance. Restart the OpMode and pick it in INIT. |
| Turret "straight ahead" is wrong / it turns too far back | Stop. Centre the turret by hand, INIT, **hold Options** to re-zero. |
| `TURRET PAST LIMIT` on the screen | The turret went past its safe range and is driving itself back. If it keeps happening, re-zero (above). |

The `Drive` line shows the assist state, e.g. `assists ON | ROBOT | snap FORWARD_BACK | heading locked 0° | line on (0.4 in off) | odometry ON`.
