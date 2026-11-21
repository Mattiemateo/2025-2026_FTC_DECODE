package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class DriveFeelTestTest {
    /** Feeds a run 100 Hz samples of a robot that speeds up to topSpeed (time constant tau) and brakes at decel. */
    private static DriveFeelTest.Measurement simulate(double[] step, double headingDeg, double topSpeed, double tau, double decel) {
        DriveFeelTest.Measurement m = new DriveFeelTest.Measurement(step[0], step[1], step[2], 0, 0, headingDeg, 0);
        boolean spin = step[2] != 0;
        // Direction the robot really moves in the field frame (robot frame: +x forward, +y left).
        double h = Math.toRadians(headingDeg), rx = step[0], ry = -step[1];
        double ux = rx * Math.cos(h) - ry * Math.sin(h), uy = rx * Math.sin(h) + ry * Math.cos(h);
        double dist = 0, v = 0, dt = 0.01;
        for (int i = 1; i < 2000 && !m.done; i++) {
            double t = i * dt;
            if (!m.braking) v = topSpeed * (1 - Math.exp(-t / tau));
            else v = Math.max(0, v - decel * dt);
            dist += v * dt;
            double x = spin ? 0 : dist * ux, y = spin ? 0 : dist * uy;
            double heading = spin ? headingDeg - dist : headingDeg; // clockwise spin: heading goes down
            double[] cmd = m.update(x, y, heading, t);
            if (!m.braking) assertArrayEquals(step, cmd, 0);
            else assertArrayEquals(new double[] {0, 0, 0}, cmd, 0);
        }
        assertTrue(m.done);
        return m;
    }

    @Test
    public void measuresTopSpeedAccelerationAndBraking() {
        DriveFeelTest.Measurement m = simulate(DriveFeelTest.STEPS[0], 30, 50, 0.15, 150);
        assertEquals(50, m.topSpeed, 1.0);
        assertEquals(150, m.brakeDecel, 15);
        assertTrue(m.accelTime > 0.3 && m.accelTime < 0.45); // 90% of top at 0.35 s, plus filter lag
        assertFalse(m.stillSpeedingUp);
    }

    @Test
    public void strafeAndBackwardRunsMeasureAlongTheirOwnDirection() {
        assertEquals(40, simulate(DriveFeelTest.STEPS[2], 0, 40, 0.15, 120).topSpeed, 1.0);   // strafe left
        assertEquals(40, simulate(DriveFeelTest.STEPS[3], 90, 40, 0.15, 120).topSpeed, 1.0);  // strafe right
        assertEquals(50, simulate(DriveFeelTest.STEPS[1], -135, 50, 0.15, 150).topSpeed, 1.0); // backward
    }

    @Test
    public void spinMeasuresTurnRateAcrossTheHeadingWrap() {
        DriveFeelTest.Measurement m = simulate(DriveFeelTest.STEPS[4], 170, 300, 0.1, 900);
        assertEquals(300, m.topSpeed, 6);
    }

    @Test
    public void flagsARunThatNeverReachedTopSpeed() {
        assertTrue(simulate(DriveFeelTest.STEPS[0], 0, 50, 1.5, 150).stillSpeedingUp);
    }

    @Test
    public void pasteLinesUseTheGameNamesAndSkipStoppedRuns() {
        DriveFeelTest.Measurement[] done = new DriveFeelTest.Measurement[5];
        done[0] = simulate(DriveFeelTest.STEPS[0], 0, 50, 0.15, 150);
        done[1] = simulate(DriveFeelTest.STEPS[1], 0, 54, 0.15, 150);
        done[4] = simulate(DriveFeelTest.STEPS[4], 0, 300, 0.1, 900);
        done[2] = new DriveFeelTest.Measurement(0, -1, 0, 0, 0, 0, 0);
        done[2].abort();
        String text = String.join("\n", DriveFeelTest.pasteLines(done));
        assertTrue(text, text.contains("maxAchievableForwardVelocity: 52."));
        assertTrue(text, text.contains("maxTurnRate: 30"));
        assertFalse(text, text.contains("maxAchievableStrafeVelocity"));
        assertTrue(text, text.contains("brakeDeceleration: "));
        assertTrue(text, text.contains("accelerationTime: 0."));
    }
}
