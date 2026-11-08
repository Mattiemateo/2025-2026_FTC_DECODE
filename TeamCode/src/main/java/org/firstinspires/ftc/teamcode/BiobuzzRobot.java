package org.firstinspires.ftc.teamcode;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/** BIOBUZZ hardware shared by the TeleOp and the shot calibration OpMode. */
public class BiobuzzRobot {
    public static double INTAKE_POWER = 0.6;
    public static double TRANSFER_POWER = 0.9;
    // 0..1: how much each odometry sample updates the velocity estimate (lower = smoother).
    public static double VELOCITY_FILTER = 0.5;

    public DcMotor frontLeft, frontRight, backLeft, backRight, intake, transfer;
    public FlywheelController flywheel;
    public final TurretController turret = new TurretController();
    public Localizer localizer;
    public String odometryError;

    private Pose lastPose;
    private long lastPoseNanos;
    private double vForward, vLeft;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
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

    /** During INIT: Circle = RED, Cross = BLUE. */
    public static void selectAlliance(Gamepad gamepad) {
        if (gamepad.circle) TurretController.ALLIANCE = TurretController.Alliance.RED;
        if (gamepad.cross) TurretController.ALLIANCE = TurretController.Alliance.BLUE;
    }

    public void drive(double forward, double strafe, double turn, double scale) {
        double[] powers = BIOBUZZ.drivePowers(forward, strafe, turn, scale);
        frontLeft.setPower(powers[0]);
        frontRight.setPower(powers[1]);
        backLeft.setPower(powers[2]);
        backRight.setPower(powers[3]);
    }

    /**
     * Updates odometry, works out the robot's velocity in its own frame and hands it to the
     * turret for shot lead. Returns the pose, or null when odometry is off.
     */
    public Pose updateOdometry() {
        if (localizer == null) {
            turret.setRobotVelocity(0, 0);
            return null;
        }
        Pose pose;
        try {
            localizer.update();
            pose = localizer.pose();
        } catch (RuntimeException e) {
            localizer = null; // untuned or broken odometry: keep driving, just no lead / field aim
            odometryError = e.getMessage();
            turret.setRobotVelocity(0, 0);
            return null;
        }
        long now = System.nanoTime();
        if (lastPose != null) {
            double dt = (now - lastPoseNanos) / 1e9;
            if (dt > 0.005) {
                double[] v = robotFrameVelocity(pose.x() - lastPose.x(), pose.y() - lastPose.y(),
                        pose.heading(), dt);
                vForward += VELOCITY_FILTER * (v[0] - vForward);
                vLeft += VELOCITY_FILTER * (v[1] - vLeft);
                lastPose = pose;
                lastPoseNanos = now;
            }
        } else {
            lastPose = pose;
            lastPoseNanos = now;
        }
        turret.setRobotVelocity(vForward, vLeft);
        return pose;
    }

    /** Field-frame displacement over dt turned into {forward, left} speed in the robot frame. */
    static double[] robotFrameVelocity(double dx, double dy, double headingRad, double dt) {
        double cos = Math.cos(headingRad), sin = Math.sin(headingRad);
        return new double[] {(dx * cos + dy * sin) / dt, (-dx * sin + dy * cos) / dt};
    }

    public void stopAll() {
        for (DcMotor motor : new DcMotor[] {frontLeft, frontRight, backLeft, backRight, intake, transfer}) {
            if (motor != null) motor.setPower(0);
        }
        if (flywheel != null) flywheel.setTargetRPM(0);
        turret.stop();
    }
}
