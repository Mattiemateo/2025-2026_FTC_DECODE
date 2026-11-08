package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Builds the ShotTable. Stand still at a distance, let auto-aim lock on, adjust the RPM until
 * shots go in, then copy the "Table row" line into ShotTable.ROWS. Repeat at 4-6 distances,
 * ideally side-on to the HIVE like in a match.
 *
 * During INIT: Circle = RED, Cross = BLUE.
 * D-pad up/down: RPM +/- RPM_STEP | Square: feed (shoot) | Circle: transfer reverse
 * Triangle: auto-aim on/off (on at start) | Bumpers: turn turret while auto-aim is off
 * Left stick / right stick X: drive slowly into position
 */
@TeleOp(name = "BIOBUZZ Shot Calibration", group = "BIOBUZZ")
public class ShotCalibration extends LinearOpMode {
    public static double RPM_STEP = 50.0;
    public static double DRIVE_SCALE = 0.4;

    private final BiobuzzRobot robot = new BiobuzzRobot();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        while (opModeInInit()) {
            BiobuzzRobot.selectAlliance(gamepad1);
            telemetry.addData("Alliance", "%s  (Circle = RED, Cross = BLUE)", TurretController.ALLIANCE);
            telemetry.addLine("Center the turret before INIT. Stand still while shooting.");
            telemetry.update();
            idle();
        }

        double rpm = FlywheelController.DEFAULT_RPM;
        boolean autoAim = true;
        boolean lastUp = false, lastDown = false, lastTriangle = false;

        try {
            while (opModeIsActive()) {
                robot.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, DRIVE_SCALE);

                if (gamepad1.dpad_up && !lastUp) rpm = Math.min(FlywheelController.MAX_RPM, rpm + RPM_STEP);
                if (gamepad1.dpad_down && !lastDown) rpm = Math.max(0, rpm - RPM_STEP);
                if (gamepad1.triangle && !lastTriangle) autoAim = !autoAim;
                lastUp = gamepad1.dpad_up;
                lastDown = gamepad1.dpad_down;
                lastTriangle = gamepad1.triangle;

                robot.turret.setRobotVelocity(0, 0); // standing still: no lead while calibrating
                robot.turret.enableAutoAim(autoAim);
                if (!autoAim) robot.turret.jogManual(BIOBUZZ.buttonAxis(gamepad1.right_bumper, gamepad1.left_bumper));
                robot.turret.update();

                robot.flywheel.setTargetRPM(rpm);
                robot.transfer.setPower(BiobuzzRobot.TRANSFER_POWER * BIOBUZZ.buttonAxis(gamepad1.square, gamepad1.circle));

                double distance = robot.turret.getGoalDistanceIn();
                telemetry.addData("Horizontal distance", distance > 0 ? String.format("%.1f in", distance) : "no goal / no 3D");
                telemetry.addData("RPM (target / measured)", "%.0f / %.0f", rpm, robot.flywheel.getCurrentRPM());
                telemetry.addData("Aim error", "%.1f°", robot.turret.getAimErrorDeg());
                telemetry.addData("Table row", "{%.1f, %.0f, %.2f},", distance, rpm, ShotTable.flightTime(distance));
                telemetry.addLine("Copy the row into ShotTable.ROWS when shots go in (flight time: measure on video).");
                telemetry.update();
            }
        } finally {
            robot.stopAll();
        }
    }
}
