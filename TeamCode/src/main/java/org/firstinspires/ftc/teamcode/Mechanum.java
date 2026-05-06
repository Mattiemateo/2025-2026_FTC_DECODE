package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "Mechanum Drive with Auto-Aiming Turret", group = "Mechanum")
public class Mechanum extends LinearOpMode {

    // --- Constants --- git pls push
    public static double MANUAL_TURRET_INCREMENT = 4.0; // degrees
    public static double HOOD_INCREMENT = 0.05;
    public static double SLOW_MODE_SCALE = 0.25;
    public static double FLIPPER_IDLE_POSITION = 0.7;
    public static double FLIPPER_LAUNCH_POSITION = 0.1;
    public static double FLYWHEEL_INTERRUPT_DURATION = 0.6; // Seconds to cut flywheel power during launch

    public static double blocker_closed = 0.3;

    public static double blocker_open = 0.6;
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

        Servo flipper = hardwareMap.get(Servo.class, "flipper"); //port 0 Control hub
        Servo blocker = hardwareMap.get(Servo.class, "blocker"); //port 1 Control hub
        Servo hood = hardwareMap.get(Servo.class, "hood"); // port 0 Expansion hub

        DigitalChannel modeSwitch = hardwareMap.get(DigitalChannel.class, "mode");

        // --- Controller Initialization ---
        TurretController turret = new TurretController();
        turret.init(hardwareMap, telemetry);

        FlywheelController flywheel = new FlywheelController();
        flywheel.init(hardwareMap, telemetry);

        modeSwitch.setMode(DigitalChannel.Mode.INPUT);

        // --- Motor & Servo Configuration ---
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
        boolean autoAim = false;
        boolean lastTrianglePressed = false;
        double hoodPosition = 0.6;;
        boolean intakeOn = false;
        boolean lastCirclePressed = false;
        boolean lastFlipperPressed = false;
        boolean redMode = false;
        com.qualcomm.robotcore.util.ElapsedTime flywheelInterruptTimer =
            new com.qualcomm.robotcore.util.ElapsedTime();
        boolean isInterruptingFlywheel = false;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        flipper.setPosition(FLIPPER_IDLE_POSITION);
        blocker.setPosition(blocker_closed);

        while (opModeIsActive()) {
            //flipper.setPosition(FLIPPER_IDLE_POSITION);
            //redMode = !modeSwitch.getState();
            redMode = true;
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
            flywheel.update();

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
            boolean invertIntake =
                gamepad1.right_bumper || gamepad2.right_bumper;
            intake.setPower(intakeOn ? invertIntake ? -0.7 : 0.7 : 0);

            // Flipper and Flywheel control with power interrupt
            boolean flipperPressed = (gamepad1.right_trigger > 0 || gamepad2.right_trigger > 0) && intakeOn;
            boolean flywheelRequested =
                    gamepad2.left_bumper || gamepad1.left_bumper;
            // Detect flipper trigger press (rising edge)

            // Detect flipper trigger press (rising edge)
            if (flipperPressed && !lastFlipperPressed) {
                // Start flywheel interrupt to give flipper more power
                isInterruptingFlywheel = true;
                flywheelInterruptTimer.reset();
            }
            lastFlipperPressed = flipperPressed;

            // Check if interrupt duration has expired
            if (
                isInterruptingFlywheel &&
                flywheelInterruptTimer.seconds() >= FLYWHEEL_INTERRUPT_DURATION
            ) {
                isInterruptingFlywheel = false;
            }

            // Flywheel control with PID
            if (flywheelRequested) {
                if (autoAim) {
                    // Auto-aim enabled: use distance-based lookup table
                    double distance = turret.getLastKnownDistance();
                    if (distance > 0) {
                        flywheel.setTargetFromDistance(distance);
                    }
                } else {
                    // Auto-aim disabled: use fixed 1500 RPM
                    flywheel.setTargetRPM(1100.0);
                }
                flywheel.setEnabled(true);
            } else {
                flywheel.setEnabled(false);
            }

            // Handle power interrupt for flipper
            if (isInterruptingFlywheel) {
                flywheel.interruptPower();
            } else if (flywheelRequested) {
                flywheel.resumePower();
            }

            // Set flipper position
            if (flipperPressed) {
                blocker.setPosition(blocker_open);
                flipper.setPosition(FLIPPER_LAUNCH_POSITION);
            } else {
                blocker.setPosition(blocker_closed);
                flipper.setPosition(FLIPPER_IDLE_POSITION);
            }


            // Manual Turret Control
            if (!autoAim) {
                if (gamepad2.dpad_left || gamepad1.dpad_left) {
                    turret.setManualTargetAngle(
                        turret.getCurrentAngle() - MANUAL_TURRET_INCREMENT
                    );
                } else if (gamepad2.dpad_right || gamepad1.dpad_right) {
                    turret.setManualTargetAngle(
                        turret.getCurrentAngle() + MANUAL_TURRET_INCREMENT
                    );
                }
            }

            // Hood Control
            if (gamepad2.dpad_up || gamepad1.dpad_up) {
                hoodPosition -= HOOD_INCREMENT;
            } else if (gamepad2.dpad_down || gamepad1.dpad_down) {
                hoodPosition += HOOD_INCREMENT;
            }
            // Clamp hood position to the valid range [0.0, 1.0]
            hoodPosition = Math.max(0.0, Math.min(0.7, hoodPosition));
            hood.setPosition(hoodPosition);

            // Reset turret to zero position
            if (gamepad2.ps || gamepad1.ps) {
                turret.reset();
            }

            if (redMode) {
                turret.setPipeline(8);
            } else {
                turret.setPipeline(7);
            }

            // --- Telemetry ---
            telemetry.addData("Hood Position", "%.2f", hoodPosition);
            telemetry.addData(
                "Target Distance",
                "%.2f m",
                turret.getLastKnownDistance()
            );
            // Flywheel and TurretController handle their own telemetry
            flywheel.updateTelemetry();
            telemetry.addData("Mode", redMode ? "RED" : "BLUE");
            telemetry.update();
        }
    }
}
