package org.firstinspires.ftc.teamcode;

import static org.junit.Assert.assertArrayEquals;
import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import org.firstinspires.ftc.teamcode.TurretController.Alliance;
import org.junit.Test;

public class BiobuzzAutoTest {
    private static double distance(double[] a, double[] b) {
        return Math.hypot(a[0] - b[0], a[1] - b[1]);
    }

    /** Tile name (A1..F6) for a RED-frame point. */
    private static String tile(double[] p) {
        return "" + (char) ('A' + (int) Math.floor((p[0] + 72) / 24)) + (1 + (int) Math.floor((p[1] + 72) / 24));
    }

    @Test
    public void blueSpotsAreRedSpotsTurnedAboutTheFieldCentre() {
        assertArrayEquals(new double[] {58, -36, 180}, AutoField.forAlliance(new double[] {-58, 36, 0}, Alliance.BLUE), 1e-9);
        assertArrayEquals(new double[] {-58, 36, 0}, AutoField.forAlliance(new double[] {-58, 36, 0}, Alliance.RED), 1e-9);
        double[] p = {-40, -62, 180};
        assertArrayEquals(p, AutoField.toRed(AutoField.forAlliance(p, Alliance.BLUE), Alliance.BLUE), 1e-9);
        // RED's upward start CELL (C3, audience side) maps onto BLUE's (D4, far side).
        assertArrayEquals(FieldAim.goal(Alliance.RED, FieldAim.Cell.AUDIENCE), AutoField.startUpCell(), 1e-9);
        double[] blue = AutoField.forAlliance(AutoField.startUpCell(), Alliance.BLUE);
        assertArrayEquals(FieldAim.goal(Alliance.BLUE, FieldAim.Cell.FAR), blue, 1e-9);
        // Tag clusters: 0 red far, 1 red audience, 2 blue audience, 3 blue far.
        assertEquals(1, AutoField.startUpCluster(Alliance.RED));
        assertEquals(0, AutoField.startDownCluster(Alliance.RED));
        assertEquals(3, AutoField.startUpCluster(Alliance.BLUE));
        assertEquals(2, AutoField.startDownCluster(Alliance.BLUE));
    }

    @Test
    public void startSpotsMatchTheTeamsDrawing() {
        double[] up = AutoField.startAtUpCell(), down = AutoField.startAtDownCell();
        // Auto 1: C1, back on the audience wall, facing the HIVE (+Y), C3 straight ahead.
        assertEquals("C1", tile(up));
        assertEquals("C3", tile(AutoField.startUpCell()));
        assertEquals(-72, up[1] - FieldAim.ROBOT_HALF_LENGTH, 1e-9);
        assertEquals(90, up[2], 1e-9);
        assertEquals(AutoField.startUpCell()[0], up[0], 1e-9);
        // Auto 2/3: C6, back on the far wall, facing the HIVE (-Y), C4 straight ahead.
        assertEquals("C6", tile(down));
        assertEquals("C4", tile(AutoField.startDownCell()));
        assertEquals(72, down[1] + FieldAim.ROBOT_HALF_LENGTH, 1e-9);
        assertEquals(-90, down[2], 1e-9);
        // Whole robot on our half (x < 0).
        assertTrue(up[0] + AutoField.ROBOT_HALF_WIDTH < 0);
        // BLUE: D6 and D1.
        assertEquals("D6", tile(AutoField.forAlliance(up, Alliance.BLUE)));
        assertEquals("D1", tile(AutoField.forAlliance(down, Alliance.BLUE)));
    }

    @Test
    public void autoOneLeavesForwardShootsAndParksInA5() {
        double[] start = AutoField.startAtUpCell();
        double[] leave = AutoField.leaveSpot(start);
        assertEquals(start[0], leave[0], 1e-9);
        assertEquals(start[1] + AutoField.LEAVE_IN, leave[1], 1e-9); // forward = toward the HIVE
        assertTrue(ShotTable.inRange(distance(leave, AutoField.startUpCell())));
        double[][] route = AutoField.parkRoute(leave[0], leave[1], leave[2]);
        double[] park = route[route.length - 1];
        assertEquals("A5", tile(park));
        assertTrue(park[0] - FieldAim.ROBOT_HALF_LENGTH < -61); // overlaps the LOADING ZONE (x -72..-61)
        // Up the lane, clear of the HIVE legs (x about -25) and the GARDEN corner (x < -48).
        assertEquals(AutoField.LANE_X, route[0][0], 1e-9);
        assertTrue(AutoField.LANE_X + AutoField.ROBOT_HALF_WIDTH < -25);
        assertTrue(AutoField.LANE_X - AutoField.ROBOT_HALF_WIDTH > -50);
        for (double[] p : route) assertEquals(90, p[2], 1e-9); // no turning on the way
    }

    @Test
    public void shotsAreFromAboutOnePointTwoMetresAndReachable() {
        double[] garden = AutoField.gardenShot(), fallback = AutoField.fallbackShot();
        assertEquals(120, distance(garden, AutoField.startDownCell()) * 2.54, 1);
        assertEquals(120, distance(fallback, AutoField.startUpCell()) * 2.54, 1);
        assertTrue(ShotTable.inRange(distance(AutoField.startAtDownCell(), AutoField.startDownCell())));
        for (double[][] shot : new double[][][] {{garden, AutoField.startDownCell()}, {fallback, AutoField.startUpCell()}}) {
            double angle = TurretController.turretAngleToPoint(shot[0][0], shot[0][1], shot[0][2], shot[1][0], shot[1][1]);
            assertTrue(angle > TurretController.MIN_ANGLE && angle < TurretController.MAX_ANGLE);
            assertTrue(shot[0][0] + AutoField.ROBOT_HALF_WIDTH < 0);
        }
        // The GARDEN shot is in the LOADING ZONE already: park from there without a detour.
        assertEquals(1, AutoField.parkRoute(garden[0], garden[1], 0).length);
    }

    @Test
    public void gardenRunPassesAllFourPollenAlongTheWall() {
        double[][] route = AutoField.gardenRoute();
        double[] start = route[route.length - 1], end = AutoField.gardenEnd();
        assertEquals(-72 + AutoField.ROBOT_HALF_WIDTH + AutoField.GARDEN_WALL_GAP_IN, start[1], 1e-9);
        assertEquals(180, start[2], 1e-9);
        // POLLEN centres x -70.6..-62.2; the robot's front (intake) must pass the last one.
        assertTrue(start[0] - FieldAim.ROBOT_HALF_LENGTH > -62.2);
        assertTrue(end[0] - FieldAim.ROBOT_HALF_LENGTH < -70.6);
        assertEquals(AutoField.LANE_X, route[0][0], 1e-9);
    }

    @Test
    public void fallbackGoesUnderTheHiveThenRightClearOfThePartner() {
        double[][] route = AutoField.fallbackRoute();
        double[] under = route[0], spot = route[route.length - 1], start = AutoField.startAtDownCell();
        // Straight ahead from C6 (same x), past both CELLs of the HIVE.
        assertEquals(start[0], under[0], 1e-9);
        assertTrue(under[1] < AutoField.startUpCell()[1]);
        // Then right (toward our wall, -X, when facing the audience).
        assertTrue(spot[0] < under[0]);
        // Clear of the partner robot in C1 (y up to -72 + 18 = -54).
        double partnerTop = -72 + 2 * FieldAim.ROBOT_HALF_LENGTH;
        assertTrue(under[1] - FieldAim.ROBOT_HALF_LENGTH > partnerTop);
        assertTrue(spot[0] + AutoField.ROBOT_HALF_WIDTH < AutoField.startAtUpCell()[0] - AutoField.ROBOT_HALF_WIDTH
            || spot[1] - AutoField.ROBOT_HALF_WIDTH > partnerTop);
    }

    @Test
    public void tipNeedsAllianceTagsInViewForAMoment() {
        BiobuzzAuto.TipWatch w = new BiobuzzAuto.TipWatch();
        double confirm = BiobuzzAuto.TIP_CONFIRM_SEC, gap = BiobuzzAuto.TIP_GAP_SEC;
        assertFalse(w.update(false, 0.0));
        assertFalse(w.update(true, 1.0)); // first sighting
        assertFalse(w.update(false, 1.0 + gap / 2)); // a missed frame
        assertTrue(w.update(true, 1.0 + confirm)); // still counts from the first sighting
        BiobuzzAuto.TipWatch w2 = new BiobuzzAuto.TipWatch();
        assertFalse(w2.update(true, 0.0)); // a stray detection...
        assertFalse(w2.update(false, gap + 0.1)); // ...then nothing for longer than the gap
        assertFalse(w2.update(true, gap + 0.2)); // starts over
        assertFalse(w2.update(true, gap + 0.2 + confirm / 2));
        assertTrue(w2.update(true, gap + 0.2 + confirm));
    }

    @Test
    public void startHintNamesTheTile() {
        assertTrue(BiobuzzAuto.startHint(BiobuzzAuto.Plan.HIVE_FRONT, Alliance.RED).contains("C1"));
        assertTrue(BiobuzzAuto.startHint(BiobuzzAuto.Plan.HIVE_FRONT, Alliance.RED).contains("audience"));
        assertTrue(BiobuzzAuto.startHint(BiobuzzAuto.Plan.HIVE_FRONT, Alliance.BLUE).contains("D6"));
        assertTrue(BiobuzzAuto.startHint(BiobuzzAuto.Plan.SECOND_CELL_TIP, Alliance.RED).contains("C6"));
        assertTrue(BiobuzzAuto.startHint(BiobuzzAuto.Plan.SECOND_CELL_PARK, Alliance.BLUE).contains("D1"));
    }
}
