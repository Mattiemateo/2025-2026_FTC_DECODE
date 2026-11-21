# BIOBUZZ Driver Practice

A 2D top-down practice game for the **BIOBUZZ** TeleOp. It runs in a browser and uses a PS4 / PS5 controller, with the same controls as `BIOBUZZ.java` on the `biobuzz-season` branch.

## Play

1. Connect a PS4 (DualShock 4) or PS5 (DualSense) controller to the Mac over USB or Bluetooth.
2. Open `index.html` in **Chrome** (recommended) or Safari. Double-clicking the file works. No install or internet needed.
3. Press any button on the controller so the browser detects it. The dot in the side panel turns green.
4. **INIT:** Circle = RED, Cross = BLUE (as on the robot). Then press the **touchpad** (or the PS button, or Enter) to start the match.

Hold the touchpad (or Enter) for 1 second during a match to go back to INIT. On the real robot the Driver Station starts the OpMode, so this is a simulator-only button; Share now does what it does on the robot.

## Controls (same as the TeleOp)

The driver cheat sheet, [`doc/driver-cheatsheet.md`](../doc/driver-cheatsheet.md), applies to the sim as well.

| Controller | Action | Keyboard |
| --- | --- | --- |
| Left stick | Drive / strafe (80%), with the driving assists | W A S D |
| Right stick X | Turn | ← → |
| L3 (press) | Field-relative driving on/off: stick forward = away from the drivers. The HUD shows FIELD-/ROBOT-RELATIVE, and a blue arrow at the robot points where "stick forward" goes | G |
| R3 (press) | "Away from the drivers" = the way the robot faces now | H |
| R2 / L2 | Intake in / out (60%) | I / K |
| R1 / L1 | Turret clockwise / counter-clockwise (80°/s, 30°/s near a limit; ignored while auto-aim is on) | E / Q |
| Triangle | Toggle auto-aim | R |
| Square | Auto-aim ON: hold to shoot (fires only when every check passes). OFF: flywheel 1800 RPM, right for about 1 m | F / Space |
| Cross / Circle | Transfer forward / reverse | Z / X |
| D-pad ↑ / ↓ | FAR / AUDIENCE CELL is up (odometry aim) | ↑ / ↓ |
| D-pad ← / → | Shot speed trim −/+ 25 RPM on every shot. Kept until the page reloads, like the robot keeps it until its app restarts | [ / ] |
| Options | Re-zero odometry at the start spot (also forgets the remembered goal) | O |
| Share, tap | Odometry off/on: no line hold, goal memory, shot lead or odometry aim | V |
| Share, hold 0.6 s | Driving assists off/on | V (hold) |
| (rumble) | A 200 ms buzz when the shot turns READY with auto-aim on and Square not held, as on the robot. The HUD's READY badge pulses too, in case the browser can't rumble the controller. | |
| Touchpad | Start / stop the match (simulator only) | Enter |

**Controller check:** while you hold buttons, the controller line in the side panel lists the button numbers the browser reports. L3 must show as 10 and R3 as 11. If they don't, the browser isn't using the standard layout: use Chrome.

**Driving assists** (ported from `DriveAssist.java`, on by default):
- **Straight zone:** within 8° of pure forward/back or left/right, the stick snaps straight.
- **Response curve:** gives finer slow control.
- **Heading hold:** uses the gyro.
- **Line hold:** uses odometry.
- **Telemetry:** the `Drive` line shows the state, as on the Driver Station.

**Drivetrain drift (sim only):** the sim's robot drifts a little sideways and twists when driving straight, like the real robot does. Otherwise the assists would have nothing to correct.
- The amounts are guesses in `SIM.driveDrift`: about 1 in. sideways and 1° of twist with the assists on over a 1.2 s full-speed drive, about 3 in. and 6° with them off.
- Settings → *Drivetrain drift* → Off makes the robot drive perfectly straight.

**Goal memory:**
- Every camera sighting stores the goal in odometry coordinates.
- Closer than 45 in., or with the goal out of view, the turret aims at the stored goal. It can fire for 15 s after the last sighting, so close shots work down to the shot table's 22 in.
- `Tracking Status` and `Goal memory` show in telemetry.

## What matches the robot code

Taken from `BIOBUZZ.java`, `BiobuzzRobot.java`, `TurretController.java`, `ShotTable.java`, `FieldAim.java` and `FlywheelController.java`:

- `drivePowers` mecanum math, `DRIVE_SCALE`, `INTAKE_POWER`, `TRANSFER_POWER`
- Turret limits −170° to +110° (clockwise positive), 80°/s jog, holds position on release
- Limelight auto-aim with the gyro hold (2 s), the search sweep, the shot lead for a moving robot, and the 45 in. minimum camera distance
- `ShotTable.ROWS` for flywheel RPM and flight time; the auto-fire checks in `shotBlocker` (blind spot, goal seen, distance in table, 2° aim, 3% RPM), shown as the `Shot` telemetry line
- Field coordinates, HIVE / CELL positions and alliance start spots from `FieldAim`
- Odometry aim (Settings): the turret aims at the CELL picked with the D-pad. As on the robot, auto-fire never passes in that mode because it needs a Limelight distance.

The field and game pieces follow the Competition Manual (TU09) in the `FIELD` block. Guesses are in the `SIM` block: flywheel spin-up and RPM drop per shot. **If the Java constants change, update the matching blocks at the top of the `<script>` in `index.html`.**

## Field, game pieces and scoring

These follow the BIOBUZZ Competition Manual (TU09):

- **Game pieces:** 40 POLLEN (2.8 in.), plus 8 red and 8 blue NECTAR (3.6 in.).
- **Staging:** 4 POLLEN in each FLOWER, 4 in each GARDEN and 4 pre-loaded per robot. Each upward CELL starts with 3 NECTAR, and each ALLIANCE AREA holds 5 NECTAR.
- **Missing robots:** practice has no partner or opponents, so their pre-loads sit in the LOADING ZONES, as the manual does for robots that don't show up.
- **Robot limit:** 4 game pieces at a time (G407). Our robot only collects POLLEN.
- **Collecting from a FLOWER:** POLLEN comes out of the bottom when you drive the intake into it.
- **HIVE:**
  - Robots can drive under the HIVES. The frame triangles stand at x = ±24.7 in.
  - The CELL opening is 20 × 12 in. seen from above.
  - A tip spills the CELL under itself and turns the other CELL up, so the target switches ends.
- **Tipping (third-party rules summary, check it at the practice field):** 7 POLLEN don't tip, 8 do. 3 NECTAR + 2 POLLEN don't, 3 NECTAR + 3 POLLEN do. The game weighs NECTAR as 1.8 POLLEN, with a threshold of 7.5.
- **Belly ball (real-robot quirk, Settings):** the robot holds 4 but can only shoot 3; the last one stays in. It is drawn faded on the robot, and you can still spit it out with L2. Set *Belly ball* to *Fixed* to see what fixing it would gain.
- **Tip time:** a tip reportedly takes 2–3 s. While the HIVE swings, balls don't go in, the Limelight loses the tags, and the 20 points count when it lands.
- **Human player:** after each of our tips, one NECTAR goes into our LOADING ZONE. All remaining NECTAR enters at 1:00.
- **Points for your robot:**

  | Action | Points |
  | --- | --- |
  | TIP | 20 |
  | Ball left in the upward CELL at the end | 2 each |
  | GARDEN | 1 each |
  | PARK in the LOADING ZONE | 5 |

  FLOWERS and ranking points aren't scored.
- **Start from:**
  - *After a 1-tip auto:* counts 28 auto points (LEAVE + PARK + TIP). The robot starts parked in the LOADING ZONE.
  - *Match setup:* the robot holds its 4 pre-loads.

The side panel compares your score with the strategy benchmarks for your own robot: 13 (must-have), 35 (top-4 seed) and 70 (Energizers level). It also shows the average trip time, where a trip ends when the robot fires its last ball; the goal is about 20 s.

## Other robots (NPCs)

Set *Other robots* in Settings to add a partner and two opponents. They are simple bots, not models of real teams:

- **What they do:**
  - They collect POLLEN off the floor and out of FLOWERS, holding at most 4.
  - They drive around the HIVE frame and keep a little distance from other robots.
  - They line up about 55 in. from their own upward CELL and shoot.
  - With 12 s left they park in their LOADING ZONE.
- **Where they start:** with NPCs on, every robot starts with its own pre-loads (none sit in the LOADING ZONES). In *After a 1-tip auto*, the partner has shot its pre-loads into our new upward CELL, and the opponents' auto mirrors ours.
- **Robots push each other:** all robots, including yours, collide and push each other.
- **Opponents' human player:** with NPCs on, the opponents' human player also adds NECTAR.

| Level | Driving | Pace (fast-forward test, full TeleOp) |
| --- | --- | --- |
| Slow | 35% of top speed, misses often | about 1 tip per alliance, alliance scores 40–70 |
| Quick | 50% of top speed, accurate | a lone partner manages 2–3 tips; an opponent pair 4–5 |

The levels are tuned to the Roermond numbers in the team strategy: about 1 teleop tip for a weak robot, 2–3 for a top robot. Change `NPC_LEVELS` in `index.html` to make them easier or harder.

**Scoring with NPCs:**
- **Alliance scores:** shown in the HUD (top right) and the side panel. Winning qualification scores were about 65–70.
- **Your robot's points:** only your share counts. Each TIP is split by how many balls in the CELL were yours, and you score CELL points only for your own balls.

## Tuning the driving to the real robot

Driving speed, braking and turning come from the `DRIVE_DEFAULTS` block at the top of the script. Its names match the Pedro tuner output. The speeds are still guesses; the odometry values are the tuned ones from 2026-11-19.

**Quickest way to match the robot:** run **BIOBUZZ Drive Feel Test** on the robot. It needs about 4 tiles of clear floor around the start spot; press Cross for each of its 5 full-power runs. Then copy its last telemetry lines (`maxAchievableForwardVelocity: …`, `brakeDeceleration: …` and so on) into *Robot calibration* and press Apply. If a run says *STILL SPEEDING UP*, raise `RUN_IN` (or `SPIN_SEC` for the spin) and run it again.

You can also update the numbers in one of two ways:

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

**Odometry:** the robot uses two goBILDA 4-Bar pods plus the Control Hub IMU, with no Pinpoint. The pods were tuned on 2026-11-19 (Two Wheel Tuner plus the Odometry Tile Test): forward 0.0019494 and strafe 0.0018892 in/tick.
- **Calibrated pods (the default *Odometry drift* setting):** the simulated pods count like the real ones measured in the tile test, and are read through your ticks-to-inches values. With the tuned values, odometry only drifts by the IMU. Paste a wrong value, such as the old 0.00357 strafe, to see it throw off odometry aim and the shot lead.
- **Pod offsets:** not simulated. The tile test showed turns in place were right.
- **Old saved values:** browsers that saved the old 2026-11-07 ticks-to-inches switch to the new values automatically; other saved values stay.

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
