package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Builds the ShotTable by hand: aim the turret yourself, enter the distance you measured, adjust
 * the RPM until shots go in, and save the row. The camera is not needed.
 *
 * Distance = horizontal floor distance from the turret to the point right under the middle of the
 * goal's tag sticker (that is what the robot measures in a match), in inches.
 *
 * During INIT: Circle = RED, Cross = BLUE (only matters if you use auto-aim or the camera check).
 * L1 / R1: turn the turret | Triangle: auto-aim on/off (starts OFF)
 * D-pad up / down: RPM +/- RPM_STEP (flywheel starts OFF; hold to repeat)
 * D-pad left / right: distance -/+ DISTANCE_STEP_IN (hold to repeat)
 * Square: feed (shoot) | Circle: transfer reverse | R2 / L2: intake in / out
 * Cross: save this row (distance + measured RPM) | Options: take the distance from the camera
 * Left stick / right stick X: drive slowly into position
 */
@TeleOp(name = "BIOBUZZ Shot Calibration", group = "BIOBUZZ")
public class ShotCalibration extends LinearOpMode {
    public static double RPM_STEP = 50.0;
    public static double DISTANCE_STEP_IN = 2.0;
    public static double START_DISTANCE_IN = 58.0;
    public static double DRIVE_SCALE = 0.4;
    // Holding a D-pad button repeats its step: first repeat after REPEAT_DELAY_SEC, then every REPEAT_EVERY_SEC.
    public static double REPEAT_DELAY_SEC = 0.4;
    public static double REPEAT_EVERY_SEC = 0.1;

    private final BiobuzzRobot robot = new BiobuzzRobot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        while (opModeInInit()) {
            BiobuzzRobot.selectAlliance(gamepad1);
            telemetry.addData("Alliance", "%s  (Circle = RED, Cross = BLUE)", TurretController.ALLIANCE);
            robot.turretInitCheck(gamepad1, telemetry);
            telemetry.addLine("Stand still while shooting.");
            telemetry.addLine("Measure distance: turret to the floor point under the tag sticker's middle.");
            telemetry.update();
            idle();
        }

        double rpm = 0; // flywheel off until D-pad up
        double distance = START_DISTANCE_IN;
        double measuredRpm = 0; // smoothed: the table must hold the speed the flywheel really ran at
        boolean autoAim = false;
        boolean lastTriangle = false, lastCross = false, lastOptions = false;
        Repeat up = new Repeat(), down = new Repeat(), left = new Repeat(), right = new Repeat();
        List<String> saved = new ArrayList<>();

        try {
            while (opModeIsActive()) {
                robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, DRIVE_SCALE);
                robot.intake.setPower(BiobuzzRobot.INTAKE_POWER * (gamepad1.right_trigger - gamepad1.left_trigger));

                double now = getRuntime();
                if (up.fires(gamepad1.dpad_up, now)) rpm = Math.min(FlywheelController.MAX_RPM, rpm + RPM_STEP);
                if (down.fires(gamepad1.dpad_down, now)) rpm = Math.max(0, rpm - RPM_STEP);
                if (right.fires(gamepad1.dpad_right, now)) distance += DISTANCE_STEP_IN;
                if (left.fires(gamepad1.dpad_left, now)) distance = Math.max(0, distance - DISTANCE_STEP_IN);
                if (gamepad1.triangle && !lastTriangle) autoAim = !autoAim;
                double cameraDistance = robot.turret.getGoalDistanceIn();
                if (gamepad1.options && !lastOptions && cameraDistance > 0) distance = Math.round(cameraDistance);

                lastTriangle = gamepad1.triangle;
                lastOptions = gamepad1.options;

                robot.turret.setRobotVelocity(0, 0); // standing still: no lead while calibrating
                robot.turret.enableAutoAim(autoAim);
                if (!autoAim) robot.turret.jogManual(BIOBUZZ.buttonAxis(gamepad1.right_bumper, gamepad1.left_bumper));
                robot.turret.update();

                robot.flywheel.setTargetRPM(rpm);
                robot.transfer.setPower(BiobuzzRobot.TRANSFER_POWER * BIOBUZZ.buttonAxis(gamepad1.square, gamepad1.circle));
                measuredRpm += 0.1 * (robot.flywheel.getCurrentRPM() - measuredRpm);

                String row = tableRow(distance, measuredRpm, ShotTable.flightTime(distance));
                if (gamepad1.cross && !lastCross) {
                    saved.add(row);
                    RobotLog.i("ShotCalibration row: " + row);
                }
                lastCross = gamepad1.cross;

                telemetry.addData("Distance (your tape)", "%.0f in   (D-pad left/right)", distance);
                telemetry.addData("Camera distance", cameraDistance > 0
                        ? String.format(Locale.US, "%.0f in  (Options: use it)", cameraDistance)
                        : "not seen (fine - use the tape)");
                telemetry.addData("RPM target / measured avg", "%.0f / %.0f   (D-pad up/down, hold to repeat)%s",
                        rpm, measuredRpm, rpm == 0 ? "  - flywheel OFF" : "");
                telemetry.addData("Battery", "%.1f V", robot.batteryVoltage());
                telemetry.addData("Turret", "%s, %.0f°", autoAim ? "auto-aim" : "manual (L1/R1)",
                        robot.turret.getCurrentAngle());
                telemetry.addData("Row now", row + "   (Cross: save)");
                telemetry.addLine("Saved rows - paste into ShotTable.ROWS (sorted by distance):");
                for (String s : saved) telemetry.addLine("    " + s);
                telemetry.update();
            }
        } finally {
            robot.stopAll();
        }
    }

    /** Press = one step; hold = repeat after REPEAT_DELAY_SEC, then every REPEAT_EVERY_SEC. */
    static final class Repeat {
        private boolean wasDown = false;
        private double nextAt = 0;

        boolean fires(boolean down, double now) {
            boolean fire = false;
            if (down && !wasDown) {
                fire = true;
                nextAt = now + REPEAT_DELAY_SEC;
            } else if (down && now >= nextAt) {
                fire = true;
                nextAt = now + REPEAT_EVERY_SEC;
            }
            wasDown = down;
            return fire;
        }
    }

    /** One ShotTable.ROWS line: {distance, measured RPM, flight time}. Flight time: measure on video. */
    static String tableRow(double distanceIn, double rpm, double flightTimeSec) {
        return String.format(Locale.US, "{%.1f, %.0f, %.2f},", distanceIn, rpm, flightTimeSec);
    }
}
