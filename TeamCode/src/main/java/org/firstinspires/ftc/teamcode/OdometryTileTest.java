package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Checks the odometry on a 3x3 tile area by driving on the odometry alone.
 *
 * Setup: robot centred on a CORNER tile of the 3x3, facing along one edge, with two tiles
 * ahead of it and two tiles to its LEFT. Tile centres are 24 in apart.
 *
 * Steps (press Cross to start each one, Circle to stop the robot at any time):
 *   1. forward 48 in        -> should stop on the centre of the far corner tile
 *   2. strafe left 48 in    -> next corner
 *   3. backward 48 in       -> next corner
 *   4. strafe right 48 in   -> back on the start tile
 *   5. diagonal to the opposite corner and 6. back again
 *   7-10. turn in place to 90, 180, 270, 0 deg -> should stay on the start tile
 * After each step, measure how far the robot's centre is from the tile centre.
 *
 * Reading the result: if the robot drives FURTHER than 48 in, that pod's ticks-to-inches is
 * too small: multiply it by (real distance / 48). If it stops SHORT, it is too large (same
 * formula). Forward/backward legs test forwardTicksToInches, the strafe legs strafeTicksToInches.
 * If the robot wanders off the start tile while only turning, the pod offsets are off.
 */
@TeleOp(name = "BIOBUZZ Odometry Tile Test", group = "BIOBUZZ")
public class OdometryTileTest extends LinearOpMode {
    public static double LEG_IN = 48.0; // two tiles
    public static double MAX_POWER = 0.35;
    public static double MIN_POWER = 0.08; // enough to overcome friction near the target
    public static double KP_DRIVE = 0.04; // power per inch of error
    public static double KP_TURN = 0.015; // power per degree of error
    public static double POSITION_TOLERANCE_IN = 0.75;
    public static double HEADING_TOLERANCE_DEG = 2.0;
    public static double SETTLE_SEC = 0.3;
    public static double STEP_TIMEOUT_SEC = 8.0;

    private final BiobuzzRobot robot = new BiobuzzRobot();

    /** {x, y, headingDeg} per step, odometry inches; +x = forward at start, +y = left. Names in NAMES. */
    static double[][] waypoints(double leg) {
        return new double[][] {
            {leg, 0, 0},
            {leg, leg, 0},
            {0, leg, 0},
            {0, 0, 0},
            {leg, leg, 0},
            {0, 0, 0},
            {0, 0, 90},
            {0, 0, 180},
            {0, 0, -90},
            {0, 0, 0},
        };
    }

    static final String[] NAMES = {
        "forward 2 tiles", "strafe left 2 tiles", "backward 2 tiles", "strafe right 2 tiles",
        "diagonal to the far corner", "diagonal back to start",
        "turn to 90 (left)", "turn to 180", "turn to 270", "turn back to 0",
    };

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        while (opModeInInit()) {
            telemetry.addLine("Robot centred on a CORNER tile, facing along an edge,");
            telemetry.addLine("with 2 tiles ahead and 2 tiles to its LEFT.");
            if (robot.localizer == null) telemetry.addData("Odometry OFF", robot.odometryError);
            telemetry.update();
            idle();
        }
        if (robot.localizer == null) return;
        robot.localizer.setPose(new Pose(0, 0, 0));

        double[][] steps = waypoints(LEG_IN);
        int step = -1; // -1 = waiting to start step 0
        boolean driving = false;
        boolean lastCross = false;
        ElapsedTime stepTimer = new ElapsedTime();
        ElapsedTime settleTimer = new ElapsedTime();
        String result = "";

        try {
            while (opModeIsActive()) {
                Pose pose = robot.updateOdometry();
                if (pose == null) {
                    robot.drive(0, 0, 0, 0);
                    telemetry.addData("Odometry OFF", robot.odometryError);
                    telemetry.update();
                    continue;
                }
                double headingDeg = Math.toDegrees(pose.heading());

                boolean crossPressed = gamepad1.cross && !lastCross;
                lastCross = gamepad1.cross;
                if (gamepad1.circle) driving = false; // stop now
                if (!driving && crossPressed && step + 1 < steps.length) {
                    step++;
                    driving = true;
                    stepTimer.reset();
                    settleTimer.reset();
                    result = "";
                }

                if (driving) {
                    double[] t = steps[step];
                    double[] cmd = driveCommand(pose.x(), pose.y(), headingDeg, t[0], t[1], t[2]);
                    robot.drive(cmd[0], cmd[1], cmd[2], 1.0);
                    boolean arrived = Math.hypot(t[0] - pose.x(), t[1] - pose.y()) < POSITION_TOLERANCE_IN
                            && Math.abs(TurretController.wrap180(t[2] - headingDeg)) < HEADING_TOLERANCE_DEG;
                    if (!arrived) settleTimer.reset();
                    if (settleTimer.seconds() > SETTLE_SEC) {
                        driving = false;
                        result = "arrived - measure now";
                    } else if (stepTimer.seconds() > STEP_TIMEOUT_SEC) {
                        driving = false;
                        result = "TIMED OUT before arriving";
                    }
                } else {
                    robot.drive(0, 0, 0, 0);
                }
                robot.turret.update(); // keep the turret holding its position

                if (step < 0) {
                    telemetry.addLine("Press Cross to start step 1: " + NAMES[0]);
                } else {
                    double[] t = steps[step];
                    telemetry.addData("Step", "%d/%d %s %s", step + 1, steps.length, NAMES[step],
                            driving ? "(driving)" : result);
                    telemetry.addData("Target (x, y, h)", "%.1f, %.1f, %.0f", t[0], t[1], t[2]);
                    if (!driving && step + 1 < steps.length) {
                        telemetry.addLine("Cross: next step - " + NAMES[step + 1]);
                    } else if (!driving) {
                        telemetry.addLine("Done. Robot should be back on the start tile, facing the start way.");
                    }
                }
                telemetry.addData("Odometry (x, y, h)", "%.1f in, %.1f in, %.1f deg", pose.x(), pose.y(), headingDeg);
                telemetry.addLine("Circle: stop the robot");
                telemetry.update();
            }
        } finally {
            robot.stopAll();
        }
    }

    /**
     * Drive powers {forward, strafeRight, turnRight} toward a field target, for BIOBUZZ.drivePowers.
     * Field frame: +x forward at the start, +y left, heading counter-clockwise in degrees.
     */
    static double[] driveCommand(double x, double y, double headingDeg, double tx, double ty, double th) {
        double h = Math.toRadians(headingDeg);
        double ex = tx - x, ey = ty - y;
        double forwardErr = ex * Math.cos(h) + ey * Math.sin(h);
        double leftErr = -ex * Math.sin(h) + ey * Math.cos(h);
        double headingErr = TurretController.wrap180(th - headingDeg);

        double forward = KP_DRIVE * forwardErr;
        double left = KP_DRIVE * leftErr;
        double translation = Math.hypot(forward, left);
        double distance = Math.hypot(forwardErr, leftErr);
        if (translation > MAX_POWER) {
            forward *= MAX_POWER / translation;
            left *= MAX_POWER / translation;
        } else if (distance > POSITION_TOLERANCE_IN && translation > 0 && translation < MIN_POWER) {
            forward *= MIN_POWER / translation;
            left *= MIN_POWER / translation;
        }
        double turnCcw = clampWithMin(KP_TURN * headingErr, Math.abs(headingErr) > HEADING_TOLERANCE_DEG);
        // drivePowers: strafe positive = right, turn positive = clockwise.
        return new double[] {forward, -left, -turnCcw};
    }

    private static double clampWithMin(double power, boolean needsMovement) {
        double magnitude = Math.min(Math.abs(power), MAX_POWER);
        if (needsMovement) magnitude = Math.max(magnitude, MIN_POWER);
        return Math.copySign(magnitude, power);
    }
}
