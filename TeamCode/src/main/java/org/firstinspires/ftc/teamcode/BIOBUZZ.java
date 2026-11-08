package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * BIOBUZZ TeleOp (gamepad 1).
 *
 * During INIT: Circle = RED alliance, Cross = BLUE alliance
 * Left stick: drive/strafe | Right stick X: turn | Triggers: intake in/out
 * Left/Right bumper: turn turret | Triangle: toggle auto-aim
 * Square: auto-aim ON = hold to shoot (feeds only when aimed and up to speed);
 *         auto-aim OFF = hold for flywheel at DEFAULT_RPM
 * Cross/Circle: transfer forward/reverse (manual, e.g. to clear a jam)
 * D-pad up/down: FAR / AUDIENCE cell is tipped up (odometry aim only)
 * Options: robot is back at its start spot, re-zero odometry there
 */
@TeleOp(name = "BIOBUZZ", group = "BIOBUZZ")
public class BIOBUZZ extends LinearOpMode {
    public static double DRIVE_SCALE = 0.8;
    // Auto-fire gate: every check must pass for the transfer to feed.
    public static double AIM_TOLERANCE_DEG = 2.0;
    public static double RPM_TOLERANCE = 0.03; // fraction of the target RPM
    public static double MAX_TARGET_AGE_SEC = 0.2;

    private final BiobuzzRobot robot = new BiobuzzRobot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);

        while (opModeInInit()) {
            BiobuzzRobot.selectAlliance(gamepad1);
            telemetry.addData("Alliance", "%s  (Circle = RED, Cross = BLUE)", TurretController.ALLIANCE);
            telemetry.addLine("Turret home = current position. Center it before INIT.");
            if (TurretController.AIM_SOURCE == TurretController.AimSource.ODOMETRY) {
                telemetry.addLine("Start: back against our alliance wall, centred, facing the HIVE.");
            }
            if (robot.localizer == null) telemetry.addData("Odometry OFF", robot.odometryError);
            telemetry.addLine("Bumpers: turret | Triangle: auto-aim | Square: shoot / flywheel");
            telemetry.update();
            idle();
        }

        boolean autoAim = false;
        boolean lastTriangle = false;
        double spinRpm = FlywheelController.DEFAULT_RPM;
        FieldAim.Cell cell = FieldAim.Cell.FAR;
        resetPoseToStart();

        try {
            while (opModeIsActive()) {
                robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, DRIVE_SCALE);
                robot.intake.setPower(BiobuzzRobot.INTAKE_POWER * (gamepad1.right_trigger - gamepad1.left_trigger));

                if (gamepad1.dpad_up) cell = FieldAim.Cell.FAR;
                if (gamepad1.dpad_down) cell = FieldAim.Cell.AUDIENCE;
                if (gamepad1.options) resetPoseToStart();
                updateFieldAim(robot.updateOdometry(), cell);

                if (gamepad1.triangle && !lastTriangle) autoAim = !autoAim;
                lastTriangle = gamepad1.triangle;
                robot.turret.enableAutoAim(autoAim);
                if (!autoAim) {
                    robot.turret.jogManual(buttonAxis(gamepad1.right_bumper, gamepad1.left_bumper));
                }
                robot.turret.update();

                double manualTransfer = BiobuzzRobot.TRANSFER_POWER * buttonAxis(gamepad1.cross, gamepad1.circle);
                if (autoAim) {
                    // Keep the flywheel at the speed for the current shot distance, so a drive-by
                    // can fire straight away. Hold the last speed while the goal is briefly lost.
                    double distance = robot.turret.getShotDistanceIn();
                    if (distance > 0) spinRpm = ShotTable.rpm(distance);
                    robot.flywheel.setTargetRPM(spinRpm);

                    String blocker = shotBlocker(robot.turret.isTargetInBlindSpot(), robot.turret.getAimErrorDeg(),
                            robot.flywheel.getCurrentRPM(), spinRpm, robot.turret.secondsSinceGoalSeen(), distance);
                    boolean fire = gamepad1.square && blocker == null;
                    robot.transfer.setPower(gamepad1.square ? (fire ? BiobuzzRobot.TRANSFER_POWER : 0) : manualTransfer);
                    telemetry.addData("Shot", blocker == null ? "READY" : "WAIT: " + blocker);
                } else {
                    robot.flywheel.setTargetRPM(gamepad1.square ? FlywheelController.DEFAULT_RPM : 0);
                    robot.transfer.setPower(manualTransfer);
                }

                telemetry.addData("Auto-aim", autoAim ? "ON" : "OFF");
                telemetry.addData("Flywheel RPM (target/measured)", "%.0f / %.0f",
                        robot.flywheel.getTargetRPM(), robot.flywheel.getCurrentRPM());
                telemetry.addData("Battery", "%.1f V", robot.batteryVoltage());
                telemetry.update();
            }
        } finally {
            robot.stopAll();
        }
    }

    /**
     * Why the robot may not shoot right now, or null when every check passes. rpm is the measured
     * flywheel speed: with a low battery it can lag the target, and only the real speed counts.
     */
    static String shotBlocker(boolean blindSpot, double aimErrorDeg, double rpm, double targetRpm,
                              double targetAgeSec, double distanceIn) {
        if (blindSpot) return "goal behind the turret's blind spot";
        if (targetAgeSec > MAX_TARGET_AGE_SEC) return "goal not seen";
        if (!ShotTable.inRange(distanceIn)) return String.format("distance %.0f in outside shot table", distanceIn);
        if (aimErrorDeg > AIM_TOLERANCE_DEG) return String.format("aiming (%.1f°)", aimErrorDeg);
        if (targetRpm <= 0 || Math.abs(rpm - targetRpm) > RPM_TOLERANCE * targetRpm) {
            return String.format("rpm %.0f/%.0f", rpm, targetRpm);
        }
        return null;
    }

    private boolean hasAlliance() {
        return TurretController.ALLIANCE != TurretController.Alliance.ANY;
    }

    private void resetPoseToStart() {
        if (robot.localizer == null || !hasAlliance()) return;
        double[] start = FieldAim.startPose(TurretController.ALLIANCE);
        try {
            robot.localizer.setPose(new Pose(start[0], start[1], Math.toRadians(start[2])));
        } catch (RuntimeException e) {
            robot.localizer = null;
            robot.odometryError = e.getMessage();
        }
    }

    /** Hand the turret the angle to our upward CELL (only used when AIM_SOURCE = ODOMETRY). */
    private void updateFieldAim(Pose pose, FieldAim.Cell cell) {
        if (pose == null) {
            robot.turret.clearFieldAim();
            telemetry.addData("Odometry OFF", robot.odometryError);
            return;
        }
        double headingDeg = Math.toDegrees(pose.heading());
        telemetry.addData("Robot Pose (in, deg)", "x %.1f  y %.1f  h %.1f", pose.x(), pose.y(), headingDeg);
        if (!hasAlliance()) {
            robot.turret.clearFieldAim();
            return;
        }
        double[] goal = FieldAim.goal(TurretController.ALLIANCE, cell);
        double[] from = FieldAim.turretPosition(pose.x(), pose.y(), headingDeg);
        robot.turret.setFieldAim(from[0], from[1], headingDeg, goal[0], goal[1]);
        if (TurretController.AIM_SOURCE == TurretController.AimSource.ODOMETRY) {
            telemetry.addData("Aim Cell / Distance", "%s / %.0f in", cell,
                    Math.hypot(goal[0] - from[0], goal[1] - from[1]));
        }
    }

    /** +1 when only {@code positive} is held, -1 when only {@code negative} is held, else 0. */
    static double buttonAxis(boolean positive, boolean negative) {
        return (positive ? 1 : 0) - (negative ? 1 : 0);
    }

    static double[] drivePowers(double forward, double strafe, double turn, double scale) {
        double denominator = Math.max(1, Math.abs(forward) + Math.abs(strafe) + Math.abs(turn));
        return new double[] {
                (forward + strafe + turn) / denominator * scale,
                (forward - strafe - turn) / denominator * scale,
                (forward - strafe + turn) / denominator * scale,
                (forward + strafe - turn) / denominator * scale
        };
    }
}
