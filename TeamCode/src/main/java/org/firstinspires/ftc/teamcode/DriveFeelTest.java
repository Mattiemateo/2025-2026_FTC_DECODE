package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Measures how the robot drives, using the calibrated odometry, so the practice game
 * (practice-sim, Settings > Robot calibration) drives like the real robot: top speed forward and
 * sideways, how quickly it gets there, how hard it stops in BRAKE, and how fast it spins.
 *
 * Needs about 4 tiles of clear floor in front, behind, left and right of the start spot (the
 * backward run and the strafe-right run bring the robot back).
 * Steps (Cross starts each one, Circle stops the robot at any time):
 *   1. forward  2. backward  3. strafe left  4. strafe right  5. spin in place (clockwise)
 * Each run uses full power (not the TeleOp's 80%; the game applies that itself), cuts power after
 * RUN_IN inches or RUN_SEC seconds, and then brakes the way the TeleOp does when the sticks are let go.
 * The last telemetry lines are ready to paste into the game.
 */
@TeleOp(name = "BIOBUZZ Drive Feel Test", group = "BIOBUZZ")
public class DriveFeelTest extends LinearOpMode {
    public static double RUN_IN = 60.0;         // cut power after this far (2.5 tiles)
    public static double RUN_SEC = 2.5;         // ...or after this long
    public static double SPIN_SEC = 2.0;
    public static double PLATEAU_SEC = 0.25;    // top speed = average over the last this-long before cutting power
    public static double SPEED_FILTER = 0.3;    // 0..1, smoothing of the speed from odometry (lower = smoother)
    public static double STOPPED_SPEED = 1.0;   // in/s (deg/s for the spin) that counts as stopped
    public static double BRAKE_TIMEOUT_SEC = 3.0;

    /** {forward, strafe (+ = right), turn (+ = clockwise)} drive powers per step, for BIOBUZZ.drivePowers. */
    static final double[][] STEPS = {{1, 0, 0}, {-1, 0, 0}, {0, -1, 0}, {0, 1, 0}, {0, 0, 1}};
    static final String[] NAMES = {"forward", "backward", "strafe left", "strafe right", "spin clockwise"};

    private final BiobuzzRobot robot = new BiobuzzRobot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        while (opModeInInit()) {
            telemetry.addLine("Clear floor: about 4 tiles in front, behind, left and right.");
            telemetry.addLine("Each step drives at FULL power, then brakes.");
            if (robot.localizer == null) telemetry.addData("Odometry OFF", robot.odometryError);
            telemetry.update();
            idle();
        }
        if (robot.localizer == null) return;

        ElapsedTime clock = new ElapsedTime();
        Measurement[] done = new Measurement[STEPS.length];
        Measurement run = null;
        int step = -1;
        boolean lastCross = false;

        try {
            while (opModeIsActive()) {
                Pose pose = robot.updateOdometry();
                if (pose == null) {
                    robot.drive(0, 0, 0, 0);
                    telemetry.addData("Odometry OFF", robot.odometryError);
                    telemetry.update();
                    continue;
                }
                double t = clock.seconds();
                double headingDeg = Math.toDegrees(pose.heading());

                boolean crossPressed = gamepad1.cross && !lastCross;
                lastCross = gamepad1.cross;
                if (run == null && crossPressed && step + 1 < STEPS.length) {
                    step++;
                    double[] s = STEPS[step];
                    run = new Measurement(s[0], s[1], s[2], pose.x(), pose.y(), headingDeg, t);
                }
                if (run != null) {
                    if (gamepad1.circle) run.abort();
                    double[] cmd = run.update(pose.x(), pose.y(), headingDeg, t);
                    robot.drive(cmd[0], cmd[1], cmd[2], 1.0);
                    if (run.done) {
                        done[step] = run;
                        run = null;
                    }
                } else {
                    robot.drive(0, 0, 0, 0);
                }
                robot.turret.update(); // keep the turret holding its position

                if (run != null) {
                    telemetry.addData("Step", "%d/%d %s: %s", step + 1, STEPS.length, NAMES[step],
                            run.braking ? "braking" : "full power");
                } else if (step + 1 < STEPS.length) {
                    telemetry.addLine("Cross: step " + (step + 2) + " - " + NAMES[step + 1] + " | Circle: stop");
                } else {
                    telemetry.addLine("Done.");
                }
                for (int i = 0; i < STEPS.length; i++) {
                    if (done[i] != null) telemetry.addData(NAMES[i], done[i].summary());
                }
                for (String line : pasteLines(done)) telemetry.addLine(line);
                telemetry.update();
            }
        } finally {
            robot.stopAll();
        }
    }

    /** Lines for the practice game's Robot calibration panel, from the steps that finished. */
    static List<String> pasteLines(Measurement[] done) {
        List<String> lines = new ArrayList<>();
        double forward = average(done, 0, 1, Field.TOP), strafe = average(done, 2, 3, Field.TOP);
        double brake = average(done, 0, 1, Field.BRAKE), accel = average(done, 0, 1, Field.ACCEL);
        double turn = average(done, 4, 4, Field.TOP);
        if (forward > 0 || strafe > 0 || turn > 0) lines.add("Paste into the practice game (Robot calibration):");
        if (forward > 0) lines.add(String.format(Locale.US, "maxAchievableForwardVelocity: %.1f", forward));
        if (strafe > 0) lines.add(String.format(Locale.US, "maxAchievableStrafeVelocity: %.1f", strafe));
        if (brake > 0) lines.add(String.format(Locale.US, "brakeDeceleration: %.0f", brake));
        if (accel > 0) lines.add(String.format(Locale.US, "accelerationTime: %.2f", accel));
        if (turn > 0) lines.add(String.format(Locale.US, "maxTurnRate: %.0f", turn));
        return lines;
    }

    enum Field { TOP, ACCEL, BRAKE }

    /** Average of one result over steps first..last that finished without being stopped (0 if none). */
    static double average(Measurement[] done, int first, int last, Field field) {
        double sum = 0;
        int n = 0;
        for (int i = first; i <= last; i++) {
            Measurement m = done[i];
            if (m == null || m.aborted) continue;
            double v = field == Field.TOP ? m.topSpeed : field == Field.ACCEL ? m.accelTime : m.brakeDecel;
            if (v > 0) {
                sum += v;
                n++;
            }
        }
        return n == 0 ? 0 : sum / n;
    }

    /**
     * One full-power run and the stop after it, measured from odometry samples. Linear runs measure
     * inches along the direction of travel, the spin measures degrees turned. No hardware inside,
     * so it can be unit tested.
     */
    static final class Measurement {
        final double forward, strafe, turn;
        final boolean spin;
        private final double ux, uy, x0, y0, t0;
        private final List<double[]> samples = new ArrayList<>(); // {seconds since start, speed} at full power
        private double lastT, lastProgress, lastHeadingDeg, turned, speed;
        private double releaseProgress, stoppedSince = -1, releaseT;
        boolean braking, done, aborted;
        double topSpeed, accelTime, stopDistance, brakeDecel;
        boolean stillSpeedingUp;

        Measurement(double forward, double strafe, double turn, double x, double y, double headingDeg, double t) {
            this.forward = forward;
            this.strafe = strafe;
            this.turn = turn;
            spin = turn != 0;
            // Direction of travel in the field frame. Robot frame: +x forward, +y left; strafe + = right.
            double h = Math.toRadians(headingDeg), rx = forward, ry = -strafe, len = Math.hypot(rx, ry);
            ux = len == 0 ? 0 : (rx * Math.cos(h) - ry * Math.sin(h)) / len;
            uy = len == 0 ? 0 : (rx * Math.sin(h) + ry * Math.cos(h)) / len;
            x0 = x;
            y0 = y;
            t0 = t;
            lastT = t;
            lastHeadingDeg = headingDeg;
        }

        void abort() {
            aborted = true;
            if (!braking) release(lastProgress, lastT);
        }

        /** Feed one odometry sample; returns the drive powers {forward, strafe, turn} to apply now. */
        double[] update(double x, double y, double headingDeg, double t) {
            double progress;
            if (spin) {
                turned += Math.abs(wrap180(headingDeg - lastHeadingDeg));
                lastHeadingDeg = headingDeg;
                progress = turned;
            } else {
                progress = (x - x0) * ux + (y - y0) * uy;
            }
            double dt = t - lastT;
            if (dt > 0.005) {
                speed += SPEED_FILTER * ((progress - lastProgress) / dt - speed);
                lastProgress = progress;
                lastT = t;
                if (!braking) samples.add(new double[] {t - t0, speed});
            }

            if (!braking) {
                boolean far = !spin && progress >= RUN_IN;
                boolean long_ = t - t0 >= (spin ? SPIN_SEC : RUN_SEC);
                if (far || long_) release(progress, t);
            }
            if (braking) {
                if (Math.abs(speed) < STOPPED_SPEED) {
                    if (stoppedSince < 0) stoppedSince = t;
                } else {
                    stoppedSince = -1;
                }
                if ((stoppedSince >= 0 && t - stoppedSince > 0.2) || t - releaseT > BRAKE_TIMEOUT_SEC) {
                    done = true;
                    stopDistance = progress - releaseProgress;
                    brakeDecel = spin ? 0 : brakeDeceleration(topSpeed, stopDistance);
                }
                return new double[] {0, 0, 0};
            }
            return new double[] {forward, strafe, turn};
        }

        private void release(double progress, double t) {
            braking = true;
            releaseProgress = progress;
            releaseT = t;
            double end = t - t0;
            topSpeed = plateau(samples, end - PLATEAU_SEC, end);
            double before = plateau(samples, end - 2 * PLATEAU_SEC, end - PLATEAU_SEC);
            stillSpeedingUp = before > 0 && topSpeed > before * 1.05;
            accelTime = timeToReach(samples, 0.9 * topSpeed);
        }

        String summary() {
            if (aborted) return "stopped early, not used";
            String unit = spin ? "deg/s" : "in/s";
            String s = String.format(Locale.US, "top %.1f %s, 90%% after %.2f s", topSpeed, unit, accelTime);
            if (!spin) s += String.format(Locale.US, ", stops in %.1f in (%.0f in/s2)", stopDistance, brakeDecel);
            if (stillSpeedingUp) s += " - STILL SPEEDING UP, raise " + (spin ? "SPIN_SEC" : "RUN_IN");
            return s;
        }
    }

    /** Average speed of the samples with time in [from, to] (0 if none). */
    static double plateau(List<double[]> samples, double from, double to) {
        double sum = 0;
        int n = 0;
        for (double[] s : samples) {
            if (s[0] >= from && s[0] <= to) {
                sum += s[1];
                n++;
            }
        }
        return n == 0 ? 0 : sum / n;
    }

    /** First time the speed reaches the given value (0 if never). */
    static double timeToReach(List<double[]> samples, double speed) {
        for (double[] s : samples) if (s[1] >= speed) return s[0];
        return 0;
    }

    /** Constant deceleration that stops from speed v in distance d: v^2 / 2d. */
    static double brakeDeceleration(double v, double d) {
        return d > 0 ? v * v / (2 * d) : 0;
    }

    static double wrap180(double angle) {
        angle = angle % 360.0;
        if (angle > 180) angle -= 360;
        if (angle <= -180) angle += 360;
        return angle;
    }
}
