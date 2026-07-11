package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Configurable
@TeleOp(name = "BIOBUZZ", group = "Mechanum")
public class BIOBUZZ extends LinearOpMode {

    // --- Constants ---
    public static double SLOW_MODE_SCALE = 0.25;
    public static boolean Y_REVERSED = false;
    public static boolean X_REVERSED = true;
    public static boolean RX_REVERSED = true;

    @Override
    public void runOpMode() {
        // --- Hardware Initialization ---
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive"); // port 2 Control hub
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive"); // port 2 Expansion hub
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive"); // port 3 Control hub
        DcMotor rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive"); // port 3 Expansion hub

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake"); // port 0 Control hub

        // --- Motor Configuration ---
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motor directions - this is a common configuration
        // You may need to reverse directions based on your robot's build
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.FORWARD);

        // --- State Variables ---
        boolean intakeOn = false;
        boolean lastAPressed = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Intake Toggle (gamepad1.a) ---
            boolean isAPressed = gamepad1.a;
            if (isAPressed && !lastAPressed) {
                intakeOn = !intakeOn;
            }
            lastAPressed = isAPressed;

            // --- Drivetrain ---
            // Standard Mecanum drive logic
            double y = Y_REVERSED
                    ? -gamepad1.left_stick_y
                    : gamepad1.left_stick_y;
            double x = X_REVERSED
                    ? -gamepad1.left_stick_x
                    : gamepad1.left_stick_x;
            double rx = RX_REVERSED
                    ? -gamepad1.right_stick_x
                    : gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(
                    Math.abs(y) + Math.abs(x) + Math.abs(rx),
                    1
            );
            double leftFrontPower = (y + x + rx) / denominator;
            double leftBackPower = (y - x + rx) / denominator;
            double rightFrontPower = (y - x - rx) / denominator;
            double rightBackPower = (y + x - rx) / denominator;

            // Slow mode
            double scale = gamepad1.left_trigger > 0.5 ? SLOW_MODE_SCALE : 1.0;

            leftFrontDrive.setPower(leftFrontPower * scale);
            leftBackDrive.setPower(leftBackPower * scale);
            rightFrontDrive.setPower(rightFrontPower * scale);
            rightBackDrive.setPower(rightBackPower * scale);

            // --- Intake ---
            boolean invertIntake = gamepad1.right_bumper;
            intake.setPower(intakeOn ? (invertIntake ? 0.7 : -0.7) : 0);

            // --- Telemetry ---
            telemetry.addData("Intake", intakeOn ? "ON" : "OFF");
            telemetry.update();
        }
    }
}