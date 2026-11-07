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

    @Test(expected = IllegalArgumentException.class)
    public void turretRejectsInvalidTargets() {
        TurretController.clampAngle(Double.NaN);
    }
}
