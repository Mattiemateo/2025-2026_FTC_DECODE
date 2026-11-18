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

These are guesses (the `SIM` block at the top of the script): robot speed and acceleration, flywheel spin-up and RPM drop per shot, HIVE footprint, ball size, capacity (3), and the CELL scoring window. **If the Java constants change, update the matching blocks at the top of the `<script>` in `index.html`.**

## Settings

Change these in the side panel:

- Driver view (your alliance wall at the bottom) or audience view
- 2:00 match or free practice
- Limelight or odometry aim
- HIVE tipping
- Odometry drift (use Options to re-zero)
- Number of balls
- Robot top speed
- Aim helpers (camera cone, aim line, landing spot): turn off for match realism

Best scores are stored in the browser.

## Troubleshooting

- **Controller not detected:** press a button after the page loads. Browsers only expose a controller after it has been used.
- **Buttons wrong in Firefox:** Firefox on macOS may not use the standard layout for PS controllers. Use Chrome or Safari.
- **Controller sleeps:** PS controllers turn off after a while over Bluetooth; press the PS button to wake it.
