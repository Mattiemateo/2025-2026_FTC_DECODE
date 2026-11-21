package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class BIOBUZZTest {
    @Test
    public void mecanumMovesAndStopsInTheExpectedDirections() {
        assertArrayEquals(new double[]{0, 0, 0, 0}, BIOBUZZ.drivePowers(0, 0, 0, 1), 1e-9);
        assertArrayEquals(new double[]{1, 1, 1, 1}, BIOBUZZ.drivePowers(1, 0, 0, 1), 1e-9);
        assertArrayEquals(new double[]{1, -1, -1, 1}, BIOBUZZ.drivePowers(0, 1, 0, 1), 1e-9);
        assertArrayEquals(new double[]{1, -1, 1, -1}, BIOBUZZ.drivePowers(0, 0, 1, 1), 1e-9);
        for (double forward : new double[]{-1, 0, 1}) {
            for (double strafe : new double[]{-1, 0, 1}) {
                for (double turn : new double[]{-1, 0, 1}) {
                    for (double power : BIOBUZZ.drivePowers(forward, strafe, turn, 0.25)) {
                        assertTrue(Math.abs(power) <= 0.25);
                    }
                }
            }
        }
    }

    @Test
    public void turretTargetsRespectBothTravelLimits() {
        assertEquals(TurretController.MIN_ANGLE, TurretController.clampAngle(-360), 0);
        assertEquals(TurretController.MAX_ANGLE, TurretController.clampAngle(360), 0);
        assertEquals(12, TurretController.clampAngle(12), 0);
        TurretController turret = new TurretController();
        turret.setManualTargetAngle(360);
        assertEquals(TurretController.MAX_ANGLE, turret.getManualTargetAngle(), 0);
    }

    @Test
    public void manualTurretJogStopsAtTravelLimits() {
        double speed = TurretController.MANUAL_SPEED_DEG_PER_SEC;
        assertEquals(speed, TurretController.manualVelocity(1, 0), 0);
        assertEquals(-speed, TurretController.manualVelocity(-1, 0), 0);
        assertEquals(0, TurretController.manualVelocity(0, 0), 0);
        assertEquals(0, TurretController.manualVelocity(1, TurretController.MAX_ANGLE), 0);
        assertEquals(0, TurretController.manualVelocity(-1, TurretController.MIN_ANGLE), 0);
        assertEquals(-speed, TurretController.manualVelocity(-1, TurretController.MAX_ANGLE), 0);
    }

    @Test
    public void autoAimCounterRotatesAgainstTheRobot() {
        boolean original = TurretController.TURRET_POSITIVE_IS_CLOCKWISE;
        try {
            // Clockwise-positive turret: robot turns 30 deg CCW, turret turns 30 deg CW to compensate.
            TurretController.TURRET_POSITIVE_IS_CLOCKWISE = true;
            double bearing = TurretController.fieldBearing(10, 0, 0);
            assertEquals(40, TurretController.turretAngleFor(bearing, 30), 1e-9);
            // Target 5 deg right of center means turn 5 deg clockwise.
            assertEquals(15, TurretController.turretAngleFor(TurretController.fieldBearing(10, 5, 0), 0), 1e-9);

            // Counter-clockwise-positive turret: the same robot turn needs the opposite turret move.
            TurretController.TURRET_POSITIVE_IS_CLOCKWISE = false;
            bearing = TurretController.fieldBearing(10, 0, 0);
            assertEquals(-20, TurretController.turretAngleFor(bearing, 30), 1e-9);
            assertEquals(5, TurretController.turretAngleFor(TurretController.fieldBearing(10, 5, 0), 0), 1e-9);

            // Heading wrap-around (179 -> -179) is a 2 deg turn, not 358.
            TurretController.TURRET_POSITIVE_IS_CLOCKWISE = true;
            bearing = TurretController.fieldBearing(0, 0, 179);
            assertEquals(2, TurretController.turretAngleFor(bearing, -179), 1e-9);
        } finally {
            TurretController.TURRET_POSITIVE_IS_CLOCKWISE = original;
        }
    }

    @Test
    public void goalTagsGroupIntoClustersPerAlliance() {
        assertEquals(0, TurretController.clusterOf(30));
        assertEquals(1, TurretController.clusterOf(37));
        assertEquals(2, TurretController.clusterOf(38));
        assertEquals(3, TurretController.clusterOf(45));
        assertEquals(-1, TurretController.clusterOf(29));
        assertEquals(-1, TurretController.clusterOf(46));
        TurretController.Alliance original = TurretController.ALLIANCE;
        try {
            TurretController.ALLIANCE = TurretController.Alliance.RED;
            assertTrue(TurretController.allianceOwnsCluster(1));
            assertFalse(TurretController.allianceOwnsCluster(2));
            TurretController.ALLIANCE = TurretController.Alliance.BLUE;
            assertFalse(TurretController.allianceOwnsCluster(0));
            assertTrue(TurretController.allianceOwnsCluster(3));
        } finally {
            TurretController.ALLIANCE = original;
        }
    }

    @Test
    public void clusterCenterIsTheSameWhicheverTagsAreVisible() {
        double[] o = TurretController.TAG_OFFSETS_M;
        // Sticker 2 m ahead, 1 m up, turned 30 deg: tag i sits at center + o[i] * axis.
        double[] center = {0.3, -1.0, 2.0};
        double[] axis = {Math.cos(Math.toRadians(30)), 0, Math.sin(Math.toRadians(30))};
        double[][] tags = new double[4][3];
        for (int i = 0; i < 4; i++) for (int k = 0; k < 3; k++) tags[i][k] = center[k] + o[i] * axis[k];

        // Only the left pair (the "two tags next to each other").
        assertArrayEquals(center, TurretController.clusterCenter(
                new double[] {o[0], o[1]}, new double[][] {tags[0], tags[1]}), 1e-9);
        // One tag from each pair.
        assertArrayEquals(center, TurretController.clusterCenter(
                new double[] {o[1], o[3]}, new double[][] {tags[1], tags[3]}), 1e-9);
        // All four.
        assertArrayEquals(center, TurretController.clusterCenter(o, tags), 1e-9);
    }

    @Test
    public void odometryAimPointsTheTurretAtOurCell() {
        boolean original = TurretController.TURRET_POSITIVE_IS_CLOCKWISE;
        try {
            TurretController.TURRET_POSITIVE_IS_CLOCKWISE = true;
            // Red start: back on the red wall, facing +X (the HIVE).
            double[] start = FieldAim.startPose(TurretController.Alliance.RED);
            assertEquals(-63, start[0], 1e-9);
            assertEquals(0, start[2], 1e-9);
            // Far red cell is ahead and to the LEFT, so a clockwise-positive turret goes negative.
            double[] far = FieldAim.goal(TurretController.Alliance.RED, FieldAim.Cell.FAR);
            double expected = -Math.toDegrees(Math.atan2(far[1] - start[1], far[0] - start[0]));
            assertEquals(expected, TurretController.turretAngleToPoint(start[0], start[1], start[2], far[0], far[1]), 1e-9);
            assertTrue(expected < 0);
            // Robot turns 10 deg left (CCW): turret turns 10 deg further clockwise to stay on target.
            assertEquals(expected + 10,
                    TurretController.turretAngleToPoint(start[0], start[1], 10, far[0], far[1]), 1e-9);
            // Blue start faces -X; the audience blue cell is ahead and to the LEFT again.
            double[] blue = FieldAim.startPose(TurretController.Alliance.BLUE);
            double[] near = FieldAim.goal(TurretController.Alliance.BLUE, FieldAim.Cell.AUDIENCE);
            assertTrue(TurretController.turretAngleToPoint(blue[0], blue[1], blue[2], near[0], near[1]) < 0);
        } finally {
            TurretController.TURRET_POSITIVE_IS_CLOCKWISE = original;
        }
    }

    @Test
    public void turretReachesBothSidesAndPicksTheNearLimitInTheGap() {
        // Limits -170..110: right side (+90) and left side (-90) are both reachable.
        assertEquals(90, TurretController.reachableAngle(90, 0), 1e-9);
        assertEquals(-90, TurretController.reachableAngle(-90, 0), 1e-9);
        assertEquals(-90, TurretController.reachableAngle(270, 0), 1e-9);
        // 170 deg (behind, right) is in the gap; -170 points closer than 110.
        assertEquals(TurretController.MIN_ANGLE, TurretController.reachableAngle(170, 0), 1e-9);
        // 130 deg is closer to the 110 limit.
        assertEquals(TurretController.MAX_ANGLE, TurretController.reachableAngle(130, 0), 1e-9);
    }

    @Test
    public void shotTableInterpolatesAndRefusesOutsideItsRange() {
        double[][] rows = {{40, 1800, 0.40}, {60, 2200, 0.50}};
        assertEquals(2000, ShotTable.interpolate(rows, ShotTable.RPM, 50), 1e-9);
        assertEquals(0.45, ShotTable.interpolate(rows, ShotTable.FLIGHT_TIME, 50), 1e-9);
        assertEquals(1800, ShotTable.interpolate(rows, ShotTable.RPM, 10), 1e-9); // held flat
        assertEquals(2200, ShotTable.interpolate(rows, ShotTable.RPM, 90), 1e-9);
        assertTrue(ShotTable.inRange(rows, 50));
        assertTrue(ShotTable.inRange(rows, 40 - ShotTable.INRANGE_MARGIN_IN));
        assertFalse(ShotTable.inRange(rows, 30));
        assertFalse(ShotTable.inRange(rows, 0));
    }

    @Test
    public void movingShotsAimBehindTheGoal() {
        boolean original = TurretController.TURRET_POSITIVE_IS_CLOCKWISE;
        try {
            TurretController.TURRET_POSITIVE_IS_CLOCKWISE = true;
            double[][] rows = {{52, 2000, 0.5}};
            // Standing still: no lead.
            double[] still = TurretController.leadSolution(90, 52, 0, 0, 1.0, rows);
            assertEquals(90, still[0], 1e-9);
            assertEquals(52, still[1], 1e-9);
            // Goal on the right, driving forward: aim further back (more clockwise) and farther.
            double[] forward = TurretController.leadSolution(90, 52, 40, 0, 1.0, rows);
            assertTrue(forward[0] > 90);
            assertTrue(forward[1] > 52);
            // Driving backward: aim toward the front instead.
            assertTrue(TurretController.leadSolution(90, 52, -40, 0, 1.0, rows)[0] < 90);
            // Goal on the left, driving forward: aim further back on the left (more negative).
            assertTrue(TurretController.leadSolution(-90, 52, 40, 0, 1.0, rows)[0] < -90);
            // Exact for one step: 40 in/s * 0.5 s = 20 in back from a goal 52 in to the right.
            assertEquals(90 + Math.toDegrees(Math.atan2(20, 52)), forward[0], 1e-6);
        } finally {
            TurretController.TURRET_POSITIVE_IS_CLOCKWISE = original;
        }
    }

    @Test
    public void rpmTrimShiftsTableAndManualShots() {
        double table = ShotTable.rpm(50), manual = BIOBUZZ.manualRpm();
        try {
            ShotTable.RPM_TRIM = -50;
            assertEquals(table - 50, ShotTable.rpm(50), 1e-9);
            assertEquals(manual - 50, BIOBUZZ.manualRpm(), 1e-9);
        } finally {
            ShotTable.RPM_TRIM = 0;
        }
    }

    @Test
    public void autoFireWaitsForEveryCheck() {
        double[] row = ShotTable.ROWS[0];
        double d = row[0], rpm = row[1];
        assertNull(BIOBUZZ.shotBlocker(false, 0.5, rpm, rpm, 0.05, d));
        assertNotNull(BIOBUZZ.shotBlocker(true, 0.5, rpm, rpm, 0.05, d)); // goal in the blind spot
        assertNotNull(BIOBUZZ.shotBlocker(false, 5.0, rpm, rpm, 0.05, d)); // not aimed
        assertNotNull(BIOBUZZ.shotBlocker(false, 0.5, rpm * 0.9, rpm, 0.05, d)); // not up to speed
        assertNotNull(BIOBUZZ.shotBlocker(false, 0.5, rpm, rpm, 1.0, d)); // goal not seen recently
        assertNotNull(BIOBUZZ.shotBlocker(false, 0.5, rpm, rpm, 0.05, d + 100)); // too far
    }

    @Test
    public void readyRumbleBuzzesOnceWhenTheShotTurnsReady() {
        assertTrue(BIOBUZZ.readyRumble(false, 10)); // just turned READY
        assertFalse(BIOBUZZ.readyRumble(false, 0.02)); // was READY last loop too: no repeat
        assertFalse(BIOBUZZ.readyRumble(false, BIOBUZZ.READY_REARM_SEC / 2)); // short flicker
        assertFalse(BIOBUZZ.readyRumble(true, 10)); // Square held: already shooting
        int original = BIOBUZZ.READY_RUMBLE_MS;
        try {
            BIOBUZZ.READY_RUMBLE_MS = 0;
            assertFalse(BIOBUZZ.readyRumble(false, 10)); // switched off
        } finally {
            BIOBUZZ.READY_RUMBLE_MS = original;
        }
    }

    @Test
    public void odometryVelocityIsTurnedIntoTheRobotFrame() {
        // Facing +Y (90 deg) and moving +Y at 10 in/s = straight forward.
        double[] v = BiobuzzRobot.robotFrameVelocity(0, 1, Math.PI / 2, 0.1);
        assertEquals(10, v[0], 1e-9);
        assertEquals(0, v[1], 1e-9);
        // Facing +X and moving +Y = to the robot's left.
        v = BiobuzzRobot.robotFrameVelocity(0, 1, 0, 0.1);
        assertEquals(0, v[0], 1e-9);
        assertEquals(10, v[1], 1e-9);
    }

    @Test
    public void tiltedCameraPointsBecomeLevelDistanceAndHeight() {
        // Goal 50 in ahead and 40 in above a camera tilted up 55 deg, 5 in to the right.
        double t = Math.toRadians(55);
        double forward = 50, up = 40, right = 5;
        double[] camera = {right, forward * Math.sin(t) - up * Math.cos(t), forward * Math.cos(t) + up * Math.sin(t)};
        double[] level = TurretController.cameraToLevel(camera, 55);
        assertEquals(right, level[0], 1e-9);
        assertEquals(forward, level[1], 1e-9);
        assertEquals(up, level[2], 1e-9);
    }

    @Test
    public void tileTestDrivesTowardItsTarget() {
        // Target ahead: drive forward, no strafe, no turn.
        double[] c = OdometryTileTest.driveCommand(0, 0, 0, 48, 0, 0);
        assertTrue(c[0] > 0);
        assertEquals(0, c[1], 1e-9);
        assertEquals(0, c[2], 1e-9);
        assertTrue(Math.hypot(c[0], c[1]) <= OdometryTileTest.MAX_POWER + 1e-9);
        // Target to the left (+y) while facing +x: strafe left (negative = left for drivePowers).
        c = OdometryTileTest.driveCommand(0, 0, 0, 0, 48, 0);
        assertTrue(c[1] < 0);
        assertEquals(0, c[0], 1e-9);
        // Facing +y (90 deg), target at +y: that is straight ahead.
        c = OdometryTileTest.driveCommand(0, 0, 90, 0, 48, 90);
        assertTrue(c[0] > 0);
        assertEquals(0, c[1], 1e-9);
        // Need to turn counter-clockwise: turn power negative (drivePowers turns clockwise for +).
        c = OdometryTileTest.driveCommand(0, 0, 0, 0, 0, 90);
        assertTrue(c[2] < 0);
        // On target: no power.
        c = OdometryTileTest.driveCommand(48, 0, 0, 48, 0, 0);
        assertEquals(0, Math.hypot(c[0], Math.hypot(c[1], c[2])), 1e-9);
    }

    @Test
    public void rememberedGoalIsWhereTheCameraSawIt() {
        boolean original = TurretController.TURRET_POSITIVE_IS_CLOCKWISE;
        try {
            TurretController.TURRET_POSITIVE_IS_CLOCKWISE = true;
            // Robot at the origin facing +x, turret turned 90 deg clockwise (right), goal straight
            // ahead of the camera 50 in away: the goal is 50 in to the robot's right (-y).
            double[] goal = TurretController.goalFromSighting(0, 0, 0, 90, 0, 50);
            assertEquals(0, goal[0], 1e-9);
            assertEquals(-50, goal[1], 1e-9);
            // Target 10 deg right of the camera's centre: turret direction plus 10 deg clockwise.
            goal = TurretController.goalFromSighting(0, 0, 0, 0, 10, 50);
            assertEquals(50 * Math.cos(Math.toRadians(-10)), goal[0], 1e-9);
            assertEquals(50 * Math.sin(Math.toRadians(-10)), goal[1], 1e-9);
            // From any pose, aiming at the remembered point gives back the sighting's turret angle.
            double[][] poses = {{10, 20, 30}, {-40, 5, 170}, {0, 0, -90}};
            for (double[] p : poses) {
                double[] g = TurretController.goalFromSighting(p[0], p[1], p[2], -60, 7, 55);
                assertEquals(-53, TurretController.turretAngleToPoint(p[0], p[1], p[2], g[0], g[1]), 1e-9);
            }
        } finally {
            TurretController.TURRET_POSITIVE_IS_CLOCKWISE = original;
        }
    }

    @Test
    public void stickShapingSnapsNearlyStraightPushesToStraight() {
        // 5 deg off straight forward: sideways part removed, full speed kept.
        double[] s = DriveAssist.shapeTranslation(Math.cos(Math.toRadians(5)), Math.sin(Math.toRadians(5)));
        assertEquals(1.0, s[0], 1e-9);
        assertEquals(0, s[1], 1e-9);
        assertEquals(DriveAssist.Snap.FORWARD_BACK, DriveAssist.snapOf(s[2]));
        // 5 deg off straight right: forward part removed.
        s = DriveAssist.shapeTranslation(Math.sin(Math.toRadians(5)), Math.cos(Math.toRadians(5)));
        assertEquals(0, s[0], 1e-9);
        assertEquals(1.0, s[1], 1e-9);
        assertEquals(DriveAssist.Snap.LEFT_RIGHT, DriveAssist.snapOf(s[2]));
        // A real diagonal stays a diagonal.
        s = DriveAssist.shapeTranslation(0.7, 0.7);
        assertEquals(s[0], s[1], 1e-9);
        assertEquals(DriveAssist.Snap.NONE, DriveAssist.snapOf(s[2]));
        // Dead zone and finer control near the centre.
        assertEquals(0, DriveAssist.shapeTranslation(0.03, 0.01)[0], 1e-9);
        assertTrue(DriveAssist.shapeAxis(0.3) < 0.3);
        assertEquals(1.0, DriveAssist.shapeAxis(1.0), 1e-9);
    }

    @Test
    public void fieldRelativeTurnsTheStickIntoTheRobotFrame() {
        // Robot turned 90 deg left (CCW) from the drivers' forward: stick forward = robot's right.
        double[] r = DriveAssist.driverToRobot(1, 0, 90);
        assertEquals(0, r[0], 1e-9);
        assertEquals(1, r[1], 1e-9);
        // Robot facing the drivers (180): stick forward = robot backward.
        r = DriveAssist.driverToRobot(1, 0, 180);
        assertEquals(-1, r[0], 1e-9);
        assertEquals(0, r[1], 1e-9);
    }

    @Test
    public void headingHoldAndLineHoldPushBack() {
        // Robot rotated 10 deg counter-clockwise past its target: turn clockwise (positive).
        assertTrue(DriveAssist.headingHoldTurn(0, 10, 0) > 0);
        assertTrue(DriveAssist.headingHoldTurn(0, -10, 0) < 0);
        // Driving along +x, drifted 2 in to the left (+y): correction points right (-y).
        double[] c = DriveAssist.lineCorrection(10, 2, 0, 0, 1, 0);
        assertEquals(0, c[0], 1e-9);
        assertTrue(c[1] < 0);
        assertEquals(2, c[2], 1e-9);
        // On the line but sliding left: also push right.
        assertTrue(DriveAssist.lineCorrection(10, 0, 30, 5, 1, 0)[1] < 0);
    }

    @Test
    public void driveAssistHoldsTheLineWhileDrivingStraight() {
        DriveAssist assist = new DriveAssist();
        // Facing +x, stick pushed forward with a little right, robot drifted 3 in left.
        assist.update(1.0, 0, 0, 0, 0, true, 0, 0, 0, 0, 0); // line starts at the origin
        double[] cmd = assist.update(1.0, 0.05, 0, 0, 0, true, 20, 3, 0, 40, 0);
        assertTrue(cmd[0] > 0); // still driving forward
        assertTrue(cmd[1] > 0); // strafing right, back toward the line
        // Assists off: the stick goes straight through.
        assist.assistsOn = false;
        cmd = assist.update(0.5, 0.05, 0, 0, 0, true, 20, 3, 0, 40, 0);
        assertEquals(0.5, cmd[0], 1e-9);
        assertEquals(0.05, cmd[1], 1e-9);
    }

    @Test
    public void calibrationRowIsReadyToPasteIntoTheShotTable() {
        assertEquals("{58.0, 2013, 0.45},", ShotCalibration.tableRow(58, 2012.6, 0.45));
    }

    @Test
    public void turretSlowsDownNearItsLimits() {
        double max = TurretController.MAX_ANGLE, min = TurretController.MIN_ANGLE;
        double zone = TurretController.LIMIT_SLOW_ZONE_DEG;
        // Auto-aim power is capped when the target is close to a limit, not in the middle.
        assertEquals(0.8, TurretController.limitZonePower(0, 0.8), 1e-9);
        assertEquals(TurretController.LIMIT_ZONE_POWER, TurretController.limitZonePower(max - 5, 0.8), 1e-9);
        assertEquals(TurretController.LIMIT_ZONE_POWER, TurretController.limitZonePower(min + 5, 0.8), 1e-9);
        // Bumpers: slower toward a nearby limit, full speed away from it, nothing past it.
        assertEquals(TurretController.LIMIT_ZONE_SPEED_DEG_PER_SEC, TurretController.manualVelocity(1, max - zone / 2), 1e-9);
        assertEquals(-TurretController.MANUAL_SPEED_DEG_PER_SEC, TurretController.manualVelocity(-1, max - zone / 2), 1e-9);
        assertEquals(-TurretController.LIMIT_ZONE_SPEED_DEG_PER_SEC, TurretController.manualVelocity(-1, min + zone / 2), 1e-9);
        assertEquals(0, TurretController.manualVelocity(1, max + 1), 1e-9);
        assertEquals(0, TurretController.manualVelocity(-1, min - 1), 1e-9);
    }

    @Test
    public void heldDpadRepeatsAfterAShortPause() {
        ShotCalibration.Repeat r = new ShotCalibration.Repeat();
        assertTrue(r.fires(true, 0.0)); // press = one step
        assertFalse(r.fires(true, 0.2)); // still waiting for the repeat delay
        assertTrue(r.fires(true, 0.45)); // repeating
        assertFalse(r.fires(true, 0.5));
        assertTrue(r.fires(true, 0.56));
        assertFalse(r.fires(false, 0.6)); // released
        assertTrue(r.fires(true, 0.7)); // new press
    }

    @Test(expected = IllegalArgumentException.class)
    public void turretRejectsInvalidTargets() {
        TurretController.clampAngle(Double.NaN);
    }
}
