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
    public void manualTurretTargetNeverRunsFarAheadOfTheTurret() {
        double lead = TurretController.MANUAL_MAX_LEAD_DEG;
        // Holding the d-pad while the turret lags must not wind the target up.
        double target = 0;
        for (int i = 0; i < 50; i++) target = TurretController.nextManualTarget(target, 0, 1, 0.1);
        assertEquals(lead, target, 1e-9);
        target = 0;
        for (int i = 0; i < 50; i++) target = TurretController.nextManualTarget(target, 0, -1, 0.1);
        assertEquals(-lead, target, 1e-9);
        // Normal tracking moves at the configured rate.
        assertEquals(TurretController.MANUAL_SPEED_DEG_PER_SEC * 0.02,
                TurretController.nextManualTarget(0, 0, 1, 0.02), 1e-9);
        // Travel limits still win.
        assertEquals(TurretController.MAX_ANGLE,
                TurretController.nextManualTarget(TurretController.MAX_ANGLE, TurretController.MAX_ANGLE, 1, 0.1), 1e-9);
    }

    @Test(expected = IllegalArgumentException.class)
    public void turretRejectsInvalidTargets() {
        TurretController.clampAngle(Double.NaN);
    }
}
