package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * BIOBUZZ TeleOp (gamepad 1).
 *
 * Left stick: drive/strafe | Right stick X: turn
 * Left/Right bumper: turn turret | Triangle: toggle auto-aim
 * D-pad up/down: the FAR / AUDIENCE cell of our HIVE is tipped up (auto-aim target)
 * Options: robot is back at its start spot, re-zero odometry there
 * During INIT: Circle = RED alliance, Cross = BLUE alliance
 * Triggers: intake in/out | Cross/Circle: transfer forward/reverse | Square: flywheel
 */
@TeleOp(name = "BIOBUZZ", group = "BIOBUZZ")
public class BIOBUZZ extends LinearOpMode {
    public static double DRIVE_SCALE = 0.8;
    public static double INTAKE_POWER = 0.6;
    public static double TRANSFER_POWER = 0.9;

    private DcMotor frontLeft, frontRight, backLeft, backRight, intake, transfer;
    private FlywheelController flywheel;
    private final TurretController turret = new TurretController();
    private Localizer localizer;
    private String odometryError;

    @Override
    public void runOpMode() {
        // init() zeroes the turret encoder, so wherever the turret sits now is home (0).
        // It also sets up the IMU, so it runs before the odometry (which reads that IMU).
        turret.init(hardwareMap, telemetry);
        turret.enableAutoAim(false);

        // Odometry before the drive motors: it may reset the pod encoders, which share drive ports.
        try {
            localizer = Constants.createLocalizer(hardwareMap);
        } catch (RuntimeException e) {
            localizer = null;
            odometryError = e.getMessage(); // e.g. Two Wheel Tuner values not filled in yet
        }
        initHardware();

        // Pick the alliance during INIT: it sets the start spot and which HIVE auto-aim targets.
        while (opModeInInit()) {
            if (gamepad1.circle) TurretController.ALLIANCE = TurretController.Alliance.RED;
            if (gamepad1.cross) TurretController.ALLIANCE = TurretController.Alliance.BLUE;
            telemetry.addData("Alliance", "%s  (Circle = RED, Cross = BLUE)", TurretController.ALLIANCE);
            telemetry.addLine("Turret home = current position. Center it before INIT.");
            telemetry.addLine("Start: back against our alliance wall, centred, facing the HIVE.");
            if (localizer == null) telemetry.addData("Odometry OFF", odometryError);
            telemetry.addLine("Bumpers: turret | Triangle: auto-aim | Square: flywheel");
            telemetry.update();
            idle();
        }

        boolean autoAim = false;
        boolean lastTriangle = false;
        FieldAim.Cell cell = FieldAim.Cell.FAR;
        resetPoseToStart();

        try {
            while (opModeIsActive()) {
                drive();

                intake.setPower(INTAKE_POWER * (gamepad1.right_trigger - gamepad1.left_trigger));
                transfer.setPower(TRANSFER_POWER * buttonAxis(gamepad1.cross, gamepad1.circle));
                flywheel.setTargetRPM(gamepad1.square ? FlywheelController.DEFAULT_RPM : 0);

                if (gamepad1.dpad_up) cell = FieldAim.Cell.FAR;
                if (gamepad1.dpad_down) cell = FieldAim.Cell.AUDIENCE;
                if (gamepad1.options) resetPoseToStart();
                updateFieldAim(cell);

                if (gamepad1.triangle && !lastTriangle) autoAim = !autoAim;
                lastTriangle = gamepad1.triangle;
                turret.enableAutoAim(autoAim);
                if (!autoAim) {
                    turret.jogManual(buttonAxis(gamepad1.right_bumper, gamepad1.left_bumper));
                }
                turret.update();

                telemetry.addData("Auto-aim", autoAim ? "ON" : "OFF");
                telemetry.addData("Bumpers (L/R)", "%b / %b", gamepad1.left_bumper, gamepad1.right_bumper);
                telemetry.addData("Flywheel RPM (target/measured)", "%.0f / %.0f",
                        flywheel.getTargetRPM(), flywheel.getCurrentRPM());
                telemetry.update();
            }
        } finally {
            stopAll();
        }
    }

    private boolean hasAlliance() {
        return TurretController.ALLIANCE != TurretController.Alliance.ANY;
    }

    private void resetPoseToStart() {
        if (localizer == null || !hasAlliance()) return;
        double[] start = FieldAim.startPose(TurretController.ALLIANCE);
        try {
            localizer.setPose(new Pose(start[0], start[1], Math.toRadians(start[2])));
        } catch (RuntimeException e) {
            localizer = null;
            odometryError = e.getMessage();
        }
    }

    /** Update odometry and hand the turret the angle to our upward CELL. */
    private void updateFieldAim(FieldAim.Cell cell) {
        if (localizer == null) {
            turret.clearFieldAim();
            telemetry.addData("Odometry OFF", odometryError);
            return;
        }
        Pose pose;
        try {
            localizer.update();
            pose = localizer.pose();
        } catch (RuntimeException e) {
            localizer = null; // untuned or broken odometry: keep driving, just no auto-aim
            odometryError = e.getMessage();
            turret.clearFieldAim();
            return;
        }
        double headingDeg = Math.toDegrees(pose.heading());
        telemetry.addData("Robot Pose (in, deg)", "x %.1f  y %.1f  h %.1f", pose.x(), pose.y(), headingDeg);
        if (!hasAlliance()) {
            turret.clearFieldAim();
            telemetry.addLine("Pick an alliance during INIT for auto-aim");
            return;
        }
        double[] goal = FieldAim.goal(TurretController.ALLIANCE, cell);
        double[] from = FieldAim.turretPosition(pose.x(), pose.y(), headingDeg);
        turret.setFieldAim(from[0], from[1], headingDeg, goal[0], goal[1]);
        telemetry.addData("Aim Cell / Distance", "%s / %.0f in", cell,
                Math.hypot(goal[0] - from[0], goal[1] - from[1]));
    }

    private void initHardware() {
        frontLeft = hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.frontLeftName.get());
        frontRight = hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.frontRightName.get());
        backLeft = hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.backLeftName.get());
        backRight = hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.backRightName.get());
        intake = hardwareMap.get(DcMotor.class, "intake");
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        DcMotorEx flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");

        frontLeft.setDirection(Constants.drivetrainConfig.frontLeftDirection.get());
        frontRight.setDirection(Constants.drivetrainConfig.frontRightDirection.get());
        backLeft.setDirection(Constants.drivetrainConfig.backLeftDirection.get());
        backRight.setDirection(Constants.drivetrainConfig.backRightDirection.get());
        intake.setDirection(DcMotor.Direction.REVERSE);
        transfer.setDirection(DcMotor.Direction.FORWARD);
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);

        for (DcMotor motor : new DcMotor[] {frontLeft, frontRight, backLeft, backRight, intake, transfer}) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel = new FlywheelController(flywheelMotor);
    }

    private void drive() {
        double[] powers = drivePowers(-gamepad1.left_stick_y, gamepad1.left_stick_x,
                gamepad1.right_stick_x, DRIVE_SCALE);
        frontLeft.setPower(powers[0]);
        frontRight.setPower(powers[1]);
        backLeft.setPower(powers[2]);
        backRight.setPower(powers[3]);
    }

    private void stopAll() {
        for (DcMotor motor : new DcMotor[] {frontLeft, frontRight, backLeft, backRight, intake, transfer}) {
            if (motor != null) motor.setPower(0);
        }
        if (flywheel != null) flywheel.setTargetRPM(0);
        turret.stop();
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
