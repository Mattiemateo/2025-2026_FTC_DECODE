package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * BIOBUZZ TeleOp (gamepad 1).
 *
 * Left stick: drive/strafe | Right stick X: turn
 * Left/Right bumper: turn turret | Triangle: toggle auto-aim
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

    @Override
    public void runOpMode() {
        initHardware();

        // init() zeroes the turret encoder, so wherever the turret sits now is home (0).
        turret.init(hardwareMap, telemetry);
        turret.enableAutoAim(false);

        telemetry.addLine("Turret home = current position. Center it before INIT.");
        telemetry.addLine("Bumpers: turret | Triangle: auto-aim | Square: flywheel");
        telemetry.update();
        waitForStart();

        boolean autoAim = false;
        boolean lastTriangle = false;
        ElapsedTime loopTimer = new ElapsedTime();

        try {
            while (opModeIsActive()) {
                double dt = Math.min(loopTimer.seconds(), 0.1);
                loopTimer.reset();

                drive();

                intake.setPower(INTAKE_POWER * (gamepad1.right_trigger - gamepad1.left_trigger));
                transfer.setPower(TRANSFER_POWER * buttonAxis(gamepad1.cross, gamepad1.circle));
                flywheel.setTargetRPM(gamepad1.square ? FlywheelController.DEFAULT_RPM : 0);

                if (gamepad1.triangle && !lastTriangle) autoAim = !autoAim;
                lastTriangle = gamepad1.triangle;
                turret.enableAutoAim(autoAim);
                if (!autoAim) {
                    turret.jogManual(buttonAxis(gamepad1.right_bumper, gamepad1.left_bumper), dt);
                }
                turret.update();

                telemetry.addData("Auto-aim", autoAim ? "ON" : "OFF");
                telemetry.addData("Flywheel RPM (target/measured)", "%.0f / %.0f",
                        flywheel.getTargetRPM(), flywheel.getCurrentRPM());
                telemetry.update();
            }
        } finally {
            stopAll();
        }
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
        transfer.setDirection(DcMotor.Direction.REVERSE);
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
