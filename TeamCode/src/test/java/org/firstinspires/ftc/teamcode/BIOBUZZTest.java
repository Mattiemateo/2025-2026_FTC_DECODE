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

    @Test(expected = IllegalArgumentException.class)
    public void turretRejectsInvalidTargets() {
        TurretController.clampAngle(Double.NaN);
    }
}
