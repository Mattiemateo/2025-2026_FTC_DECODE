# BIOBUZZ Driver Practice

A 2D top-down practice game for the **BIOBUZZ** TeleOp. It runs in a browser and uses a PS4 / PS5 controller, with the same controls as `BIOBUZZ.java` on the `biobuzz-season` branch.

## Play

1. Connect a PS4 (DualShock 4) or PS5 (DualSense) controller to the Mac over USB or Bluetooth.
2. Open `index.html` in **Chrome** (recommended) or Safari. Double-clicking the file works. No install or internet needed.
3. Press any button on the controller so the browser detects it. The dot in the side panel turns green.
4. **INIT:** Circle = RED, Cross = BLUE (as on the robot). Then press **Share** (or Enter) to start the match.

Hold Share for 1 second during a match to go back to INIT. Share is a simulator-only shortcut: the TeleOp itself doesn't use it.

## Controls (same as the TeleOp)

| Controller | Action | Keyboard |
| --- | --- | --- |
| Left stick | Drive / strafe (robot-relative, 80%) | W A S D |
| Right stick X | Turn | ← → |
| R2 / L2 | Intake in / out (60%) | I / K |
| R1 / L1 | Turret clockwise / counter-clockwise (80°/s, ignored while auto-aim is on) | E / Q |
| Triangle | Toggle auto-aim | R |
| Square | Auto-aim ON: hold to shoot (fires only when every check passes). OFF: flywheel 2000 RPM | F / Space |
| Cross / Circle | Transfer forward / reverse | Z / X |
| D-pad ↑ / ↓ | FAR / AUDIENCE CELL is up (odometry aim) | ↑ / ↓ |
| Options | Re-zero odometry at the start spot | O |

## What matches the robot code

Taken from `BIOBUZZ.java`, `BiobuzzRobot.java`, `TurretController.java`, `ShotTable.java`, `FieldAim.java` and `FlywheelController.java`:

- `drivePowers` mecanum math, `DRIVE_SCALE`, `INTAKE_POWER`, `TRANSFER_POWER`
- Turret limits −170° to +110° (clockwise positive), 80°/s jog, holds position on release
- Limelight auto-aim with the gyro hold (2 s), the search sweep, the shot lead for a moving robot, and the 45 in. minimum camera distance
- `ShotTable.ROWS` for flywheel RPM and flight time; the auto-fire checks in `shotBlocker` (blind spot, goal seen, distance in table, 2° aim, 3% RPM), shown as the `Shot` telemetry line
- Field coordinates, HIVE / CELL positions and alliance start spots from `FieldAim`
- Odometry aim (Settings): the turret aims at the CELL picked with the D-pad. As on the robot, auto-fire never passes in that mode because it needs a Limelight distance.

These are guesses (the `SIM` block at the top of the script): flywheel spin-up and RPM drop per shot, HIVE footprint, ball size, capacity (3), and the CELL scoring window. **If the Java constants change, update the matching blocks at the top of the `<script>` in `index.html`.**

## Tuning the driving to the real robot

Driving speed, braking and turning come from the `DRIVE_DEFAULTS` block at the top of the script. Its names match the Pedro tuner output, and the values are guesses until tuning is done. Once the tuners have run, update the numbers in one of two ways:

- **In the game:** open *Robot calibration* in the side panel. Paste the Foresight Tuner output, the Two Wheel Tuner output, or all of `pedro/Constants.java`, and press **Apply**. Or type the numbers in. They are saved in that browser only; **Reset to defaults** goes back to the file.
- **For everyone:** edit `DRIVE_DEFAULTS` in `index.html` and commit.

| Value | From |
| --- | --- |
| `maxAchievableForwardVelocity`, `maxAchievableStrafeVelocity` | Foresight Tuner: top speed at full power (the TeleOp drives at 80% of this) |
| `naturalForwardDeceleration`, `naturalStrafeDeceleration` | Foresight Tuner: these are measured **coasting** (FLOAT), so the game uses them only as the lowest braking value and for the strafe/forward ratio |
| `brakeDeceleration` | Measure it: the TeleOp uses BRAKE. Drive at speed *v*, let go, measure the stopping distance *d*: *v*² ÷ (2*d*) |
| `maxTurnRate` | Measure it: spin at full right stick for 5 s, count turns: turns × 360 ÷ 5 × 1.25 (the 1.25 undoes the TeleOp's 80%) |
| `accelerationTime` | Seconds from standstill to full speed (stopwatch or video) |
| `forwardTicksToInches`, `strafeTicksToInches` | Two Wheel Tuner (`TwoWheelConfig`) |

**Odometry:** the robot uses two goBILDA 4-Bar pods plus the Control Hub IMU, with no Pinpoint. With *Odometry drift* set to **From ticks-to-inches**, the simulated pods count like stock 4-Bar pods (0.001979 in/tick) and are converted with your ticks-to-inches values. A wrong value then shows up as odometry drift, which affects odometry aim and the shot lead. For example, the current `strafeTicksToInches` (0.00357) makes strafing read 1.8× too far. Heading comes from the IMU and barely drifts.

## Settings

Change these in the side panel:

- Driver view (your alliance wall at the bottom) or audience view
- 2:00 match or free practice
- Limelight or odometry aim
- HIVE tipping
- Odometry drift (use Options to re-zero)
- Number of balls
- Aim helpers (camera cone, aim line, landing spot): turn off for match realism

Best scores are stored in the browser.

## Troubleshooting

- **Controller not detected:** press a button after the page loads. Browsers only expose a controller after it has been used.
- **Buttons wrong in Firefox:** Firefox on macOS may not use the standard layout for PS controllers. Use Chrome or Safari.
- **Controller sleeps:** PS controllers turn off after a while over Bluetooth; press the PS button to wake it.
