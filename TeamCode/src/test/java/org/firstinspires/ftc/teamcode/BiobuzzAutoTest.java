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

    @Test
    public void blueSpotsAreRedSpotsTurnedAboutTheFieldCentre() {
        assertArrayEquals(new double[] {58, -36, 180}, AutoField.forAlliance(new double[] {-58, 36, 0}, Alliance.BLUE), 1e-9);
        assertArrayEquals(new double[] {-58, 36, 0}, AutoField.forAlliance(new double[] {-58, 36, 0}, Alliance.RED), 1e-9);
        double[] p = {-40, -62, 180};
        assertArrayEquals(p, AutoField.toRed(AutoField.forAlliance(p, Alliance.BLUE), Alliance.BLUE), 1e-9);
        // RED's upward start CELL (far, next to the LOADING ZONE) maps onto BLUE's (audience).
        assertArrayEquals(FieldAim.goal(Alliance.RED, FieldAim.Cell.FAR), AutoField.startUpCell(), 1e-9);
        double[] blue = AutoField.forAlliance(AutoField.startUpCell(), Alliance.BLUE);
        assertArrayEquals(FieldAim.goal(Alliance.BLUE, FieldAim.Cell.AUDIENCE), blue, 1e-9);
        // Tag clusters: 0 red far, 1 red audience, 2 blue audience, 3 blue far.
        assertEquals(0, AutoField.startUpCluster(Alliance.RED));
        assertEquals(1, AutoField.startDownCluster(Alliance.RED));
        assertEquals(2, AutoField.startUpCluster(Alliance.BLUE));
        assertEquals(3, AutoField.startDownCluster(Alliance.BLUE));
    }

    @Test
    public void startSpotsTouchTheWallAndLineUpWithTheirCell() {
        double[] up = AutoField.startAtUpCell(), down = AutoField.startAtDownCell();
        assertEquals(-72, up[0] - FieldAim.ROBOT_HALF_LENGTH, 1e-9);
        assertEquals(AutoField.startUpCell()[1], up[1], 1e-9);
        assertEquals(AutoField.startDownCell()[1], down[1], 1e-9);
        // The upward CELL is on the LOADING ZONE side (y 24..48), but the start is not in it.
        assertTrue(up[1] > down[1]);
        assertTrue(up[1] + AutoField.ROBOT_HALF_WIDTH < 24);
        assertTrue(AutoField.leaveSpot(up)[0] > up[0]);
    }

    @Test
    public void shotsAreFromAboutOnePointTwoMetresAndInsideTheShotTable() {
        double[] garden = AutoField.gardenShot(), fallback = AutoField.fallbackShot();
        assertEquals(120, distance(garden, AutoField.startDownCell()) * 2.54, 1);
        assertEquals(120, distance(fallback, AutoField.startUpCell()) * 2.54, 1);
        assertTrue(ShotTable.inRange(distance(garden, AutoField.startDownCell())));
        // Op mode 1 shoots right after leaving the wall.
        assertTrue(ShotTable.inRange(distance(AutoField.leaveSpot(AutoField.startAtUpCell()), AutoField.startUpCell())));
        // The turret can reach every shot facing the HIVE (heading 0).
        for (double[][] shot : new double[][][] {{garden, AutoField.startDownCell()}, {fallback, AutoField.startUpCell()}}) {
            double angle = TurretController.turretAngleToPoint(shot[0][0], shot[0][1], 0, shot[1][0], shot[1][1]);
            assertTrue(angle > TurretController.MIN_ANGLE && angle < TurretController.MAX_ANGLE);
        }
    }

    @Test
    public void parkSpotOverlapsTheLoadingZone() {
        double[] park = AutoField.park();
        assertTrue(park[0] - FieldAim.ROBOT_HALF_LENGTH < -61); // LOADING ZONE: x -72..-61
        assertTrue(park[1] > 24 && park[1] < 48);
        // Close by it parks straight away.
        assertEquals(1, AutoField.parkRoute(park[0], park[1] - 10).length);
        // From the audience side it goes out to the lane first.
        double[][] route = AutoField.parkRoute(-36, -54);
        assertEquals(3, route.length);
        assertEquals(AutoField.LANE_X, route[0][0], 1e-9);
    }

    @Test
    public void gardenRunPassesAllFourPollenAlongTheWall() {
        double[] start = AutoField.gardenStart(), end = AutoField.gardenEnd();
        assertEquals(-72 + AutoField.ROBOT_HALF_WIDTH + AutoField.GARDEN_WALL_GAP_IN, start[1], 1e-9);
        assertEquals(180, start[2], 1e-9);
        // POLLEN centres x -70.6..-62.2; the robot's front (intake) must pass the last one.
        assertTrue(start[0] - FieldAim.ROBOT_HALF_LENGTH > -62.2);
        assertTrue(end[0] - FieldAim.ROBOT_HALF_LENGTH < -70.6);
    }

    @Test
    public void tipNeedsTheCellHighForAMoment() {
        BiobuzzAuto.TipWatch w = new BiobuzzAuto.TipWatch();
        double high = BiobuzzAuto.TIP_HEIGHT_IN + 5, low = BiobuzzAuto.TIP_HEIGHT_IN - 10;
        assertFalse(w.update(low, 0.0));
        assertFalse(w.update(high, 1.0));
        assertFalse(w.update(Double.NaN, 2.0)); // not seen: never tips on its own
        assertTrue(w.update(high, 1.0 + BiobuzzAuto.TIP_CONFIRM_SEC)); // gap didn't reset it
        BiobuzzAuto.TipWatch w2 = new BiobuzzAuto.TipWatch();
        assertFalse(w2.update(high, 0.0));
        assertFalse(w2.update(low, 0.1)); // a low reading resets
        assertFalse(w2.update(high, 0.2));
        assertFalse(w2.update(high, 0.2 + BiobuzzAuto.TIP_CONFIRM_SEC / 2));
    }

    @Test
    public void opModeOneParksFromItsStartIntoTheLoadingZone() {
        // The bug from the first field test: the start spot must be on the LOADING ZONE side, so the
        // park route only has to cover the short distance up to the zone.
        double[] leave = AutoField.leaveSpot(AutoField.startAtUpCell());
        double[][] route = AutoField.parkRoute(leave[0], leave[1]);
        double[] last = route[route.length - 1];
        assertTrue(Math.abs(last[1] - leave[1]) < 30);
        assertTrue(last[1] > leave[1]); // the zone is to the robot's left (+Y for RED facing +X)
    }

    @Test
    public void fallbackGoesUnderTheHiveThenRightAndStaysOnOurHalf() {
        double[][] route = AutoField.fallbackRoute();
        double[] under = route[0], spot = route[route.length - 1];
        // Straight ahead from the start, under the HIVE (its CELLs span about x -22.75..-2.75).
        assertEquals(AutoField.startAtDownCell()[1], under[1], 1e-9);
        assertTrue(under[0] + FieldAim.ROBOT_HALF_LENGTH > -22.75);
        // Then right (-Y for RED facing +X), and the whole robot stays on our half (G402).
        assertTrue(spot[1] < under[1]);
        assertTrue(spot[0] + AutoField.ROBOT_HALF_WIDTH < 0);
    }

    @Test
    public void startHintNamesTheRightSide() {
        assertTrue(BiobuzzAuto.startHint(BiobuzzAuto.Plan.HIVE_FRONT, Alliance.RED).contains("LEFT"));
        assertTrue(BiobuzzAuto.startHint(BiobuzzAuto.Plan.HIVE_FRONT, Alliance.BLUE).contains("LEFT"));
        assertTrue(BiobuzzAuto.startHint(BiobuzzAuto.Plan.SECOND_CELL_TIP, Alliance.RED).contains("RIGHT"));
        assertTrue(BiobuzzAuto.startHint(BiobuzzAuto.Plan.SECOND_CELL_PARK, Alliance.BLUE).contains("RIGHT"));
    }
}
