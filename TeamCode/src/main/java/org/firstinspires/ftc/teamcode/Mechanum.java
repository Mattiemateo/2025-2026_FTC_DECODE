package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Mechanum extends LinearOpMode {
    private DcMotor flywheel;
    private DcMotor intake;
    private Servo hood;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    private TurretController turret;

    double hoodpos = 0;

    @Override
    public void runOpMode() {
        // Initialize motors and servos
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        intake = hardwareMap.get(DcMotor.class, "intake"); // port 2
        hood = hardwareMap.get(Servo.class, "hood");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel"); // port 0

        // Initialize turret controller
        turret = new TurretController();
        turret.init(hardwareMap); // port 1

        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        hood.setPosition(0.0);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update turret PID
            turret.update();

            // Driving
            double x = gamepad1.left_stick_x;
            double y = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);
            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4);
            double leftFront = power * cos + turn;
            double rightFront = power * sin - turn;
            double leftBack = power * sin + turn;
            double rightBack = power * cos - turn;

            if ((power + Math.abs(turn)) > 1) {
                leftFront /= power + Math.abs(turn);
                leftBack /= power + Math.abs(turn);
                rightFront /= power + Math.abs(turn);
                rightBack /= power + Math.abs(turn);
            }

            double scale = gamepad1.left_trigger > 0.5 ? 0.25 : 1.0;

            leftFrontDrive.setPower(leftFront * scale);
            leftBackDrive.setPower(leftBack * scale);
            rightFrontDrive.setPower(rightFront * scale);
            rightBackDrive.setPower(rightBack * scale);

            // Intake control
            if (gamepad1.circle) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }

            // Flywheel control
            if (gamepad1.cross) {
                flywheel.setPower(1);
            } else {
                flywheel.setPower(0);
            }

            // Turret control - manual angle adjustment
            if (gamepad1.dpad_left) {
                turret.setTargetAngle(turret.getCurrentAngle() + 2); // +2 degrees
            } else if (gamepad1.dpad_right) {
                turret.setTargetAngle(turret.getCurrentAngle() - 2); // -2 degrees
            }

            // Hood control
            if (gamepad1.dpad_up) {
                hoodpos = Math.max(0, Math.min(hoodpos + 0.05, 1));
                hood.setPosition(hoodpos);
            } else if (gamepad1.dpad_down) {
                hoodpos = Math.max(0, Math.min(hoodpos - 0.05, 1));
                hood.setPosition(hoodpos);
            }

            // Reset turret to zero with square button
            if (gamepad1.square) {
                turret.reset();
            }

            telemetry.addData("Turret Angle", "%.1f°", turret.getCurrentAngle());
            telemetry.addData("At Target", turret.isAtTarget() ? "YES" : "NO");
            telemetry.addData("Hood Position", hoodpos);
            telemetry.update();
        }
    }
}