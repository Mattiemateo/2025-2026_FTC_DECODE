package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "Mechanum Drive with Auto-Aiming Turret", group = "Mechanum")
public class Mechanum extends LinearOpMode {

    // --- Constants ---
    public static double MANUAL_TURRET_INCREMENT = 4.0; // degrees
    public static double HOOD_INCREMENT = 0.05;
    public static double SLOW_MODE_SCALE = 0.25;

    public static boolean Y_REVERSED = false;
    public static boolean X_REVERSED = true;
    public static boolean RX_REVERSED = false;

    @Override
    public void runOpMode() {
        // --- Hardware Initialization ---
        DcMotor leftFrontDrive = hardwareMap.get(
            DcMotor.class,
            "leftFrontDrive"
        );
        DcMotor rightFrontDrive = hardwareMap.get(
            DcMotor.class,
            "rightFrontDrive"
        );
        DcMotor leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        DcMotor rightBackDrive = hardwareMap.get(
            DcMotor.class,
            "rightBackDrive"
        );

        DcMotor intake = hardwareMap.get(DcMotor.class, "intake"); // port 2
        Servo hood = hardwareMap.get(Servo.class, "hood");
        DcMotor flywheel = hardwareMap.get(DcMotor.class, "flywheel"); // port 0

        // --- Controller Initialization ---
        TurretController turret = new TurretController();
        turret.init(hardwareMap, telemetry);

        // --- Motor & Servo Configuration ---
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set motor directions - this is a common configuration
        // You may need to reverse directions based on your robot's build
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotor.Direction.REVERSE);

        // --- State Variables ---
        boolean autoAim = false;
        boolean lastTrianglePressed = false;
        double hoodPosition = 0.0;
        boolean intakeOn = false;
        boolean lastCirclePressed = false;

        // Initialize hardware to starting positions
        hood.setPosition(hoodPosition);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // --- Auto-Aim Toggle ---
            boolean isTrianglePressed = gamepad2.triangle || gamepad1.triangle;
            if (isTrianglePressed && !lastTrianglePressed) {
                autoAim = !autoAim;
                turret.enableAutoAim(autoAim);
            }
            lastTrianglePressed = isTrianglePressed;

            // --- Intake Toggle ---
            boolean isCirclePressed = gamepad2.circle || gamepad1.circle;
            if (isCirclePressed && !lastCirclePressed) {
                intakeOn = !intakeOn;
            }
            lastCirclePressed = isCirclePressed;

            // --- Update Controllers ---
            turret.update();

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

            // --- Subsystems ---
            intake.setPower(intakeOn ? 1 : 0);
            flywheel.setPower(gamepad2.cross || gamepad1.cross ? 1 : 0);

            // Manual Turret Control
            if (!autoAim) {
                if (gamepad2.dpad_left || gamepad1.dpad_left) {
                    turret.setManualTargetAngle(
                        turret.getCurrentAngle() + MANUAL_TURRET_INCREMENT
                    );
                } else if (gamepad2.dpad_right || gamepad1.dpad_right) {
                    turret.setManualTargetAngle(
                        turret.getCurrentAngle() - MANUAL_TURRET_INCREMENT
                    );
                }
            }

            // Hood Control
            if (gamepad2.dpad_up || gamepad1.dpad_up) {
                hoodPosition += HOOD_INCREMENT;
            } else if (gamepad2.dpad_down || gamepad1.dpad_down) {
                hoodPosition -= HOOD_INCREMENT;
            }
            // Clamp hood position to the valid range [0.0, 1.0]
            hoodPosition = Math.max(0.0, Math.min(1.0, hoodPosition));
            hood.setPosition(hoodPosition);

            // Reset turret to zero position
            if (gamepad2.ps || gamepad1.ps) {
                turret.reset();
            }

            // --- Telemetry ---
            telemetry.addData("Hood Position", "%.2f", hoodPosition);
            telemetry.addData(
                "Target Distance",
                "%.2f inches",
                turret.getLastKnownDistance()
            );
            // The TurretController handles its own telemetry now.
            telemetry.update();
        }
    }
}
