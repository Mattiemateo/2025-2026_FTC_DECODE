package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class MechanumWithAprilTag extends LinearOpMode {
    private DcMotor flywheel;
    private DcMotor intake;
    private Servo hood;
    private Servo flipper; //port 5
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    private TurretController turret;
    private LimelightAprilTagDetector aprilTagDetector;

    double hoodpos = 0;
    boolean autoTrackEnabled = false;
    boolean reverseIntake = false;
    //double smootheningFactor = 0.3;

    @Override
    public void runOpMode() {
        // Initialize motors and servos
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        intake = hardwareMap.get(DcMotor.class, "intake");
        hood = hardwareMap.get(Servo.class, "hood");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        flipper = hardwareMap.get(Servo.class, "flipper");

        // Initialize turret controller
        turret = new TurretController();
        turret.init(hardwareMap);

        // Initialize AprilTag detector
        // Change this line in your TeleOp:
        aprilTagDetector = new LimelightAprilTagDetector();  // Instead of AprilTagDetector
        aprilTagDetector.init(hardwareMap);

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
        flipper.setPosition(0.8);

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Camera", "Ready");
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
            } else if (gamepad1.square) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            // Flywheel control
            if (gamepad1.cross) {
                flywheel.setPower(1);
            } else {
                flywheel.setPower(0);
            }

            if (gamepad1.left_bumper) {
                flipper.setPosition(0.5);
            } else {
                flipper.setPosition(0.8);
            }

            // Toggle auto-tracking with triangle button
            if (gamepad1.triangle) {
                autoTrackEnabled = !autoTrackEnabled;
                sleep(200); // Debounce
            }

            // Turret control
            if (autoTrackEnabled) {
                // Auto-track AprilTag
                Double bearing = aprilTagDetector.getTargetBearing();
                if (bearing != null) {
                    if (bearing <= -5) {
                        turret.setTargetAngle(turret.getCurrentAngle() - (bearing / 5));
                    } else if (bearing >= 5) {
                        turret.setTargetAngle(turret.getCurrentAngle() + (bearing / 5));
                    }
                    //if (Math.abs(bearing) > 1) {
                    //    turret.setTargetAngle((turret.getCurrentAngle() * (1 - smootheningFactor)) + (bearing * smootheningFactor));
                    //}
                }
            } else {
                // Manual control with D-pad
                if (gamepad1.dpad_left) {
                    turret.setTargetAngle(turret.getCurrentAngle() + 1);
                } else if (gamepad1.dpad_right) {
                    turret.setTargetAngle(turret.getCurrentAngle() - 1);
                }// else {
                // Lock current position when no input
                //    turret.setTargetAngle(turret.getCurrentAngle());
                //}
            }

            // Hood control
            if (autoTrackEnabled) {
                if (false) {
                    hood.setPosition(hoodpos);
                }
            } else {
                if (gamepad1.dpad_up) {
                    hoodpos = Math.max(0, Math.min(hoodpos + 0.05, 1));
                    hood.setPosition(hoodpos);
                } else if (gamepad1.dpad_down) {
                    hoodpos = Math.max(0, Math.min(hoodpos - 0.05, 1));
                    hood.setPosition(hoodpos);
                }
            }

            // Telemetry
            telemetry.addData("Auto-Track", autoTrackEnabled ? "ENABLED" : "DISABLED");
            telemetry.addData("Tag Visible", aprilTagDetector.isTagVisible() ? "YES" : "NO");

            //Double distance = aprilTagDetector.getTargetDistance();
            //if (distance != null) {
            //    telemetry.addData("Tag Distance", "%.1f inches", distance);
            //}

            Double bearing = aprilTagDetector.getTargetBearing();
            if (bearing != null) {
                telemetry.addData("Tag Bearing", "%.1f°", bearing);
            }

            telemetry.addData("Turret Angle", "%.1f°", turret.getCurrentAngle());
            telemetry.addData("At Target", turret.isAtTarget() ? "YES" : "NO");
            telemetry.addData("Hood Position", hoodpos);

            telemetry.addData("rxs", gamepad1.right_stick_x);

            telemetry.update();
        }

        // Cleanup
        aprilTagDetector.close();
    }
}