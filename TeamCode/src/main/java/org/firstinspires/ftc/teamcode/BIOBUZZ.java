package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp(name = "BIOBUZZ", group = "BIOBUZZ")
public class BIOBUZZ extends LinearOpMode {
    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.frontLeftName.get());
        DcMotor frontRight = hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.frontRightName.get());
        DcMotor backLeft = hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.backLeftName.get());
        DcMotor backRight = hardwareMap.get(DcMotor.class, Constants.drivetrainConfig.backRightName.get());
        DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        DcMotor transfer = hardwareMap.get(DcMotor.class, "transfer");
        DcMotorEx flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        DcMotor[] motors = {frontLeft, frontRight, backLeft, backRight, intake, transfer, flywheel};
        TurretController turret = new TurretController();

        try {
            for (DcMotor motor : motors) {
                motor.setPower(0);
                motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                motor.setDirection(DcMotor.Direction.FORWARD);
            }
            frontLeft.setDirection(Constants.drivetrainConfig.frontLeftDirection.get());
            frontRight.setDirection(Constants.drivetrainConfig.frontRightDirection.get());
            backLeft.setDirection(Constants.drivetrainConfig.backLeftDirection.get());
            backRight.setDirection(Constants.drivetrainConfig.backRightDirection.get());
            intake.setDirection(DcMotor.Direction.REVERSE);
            transfer.setDirection(DcMotor.Direction.REVERSE);
            flywheel.setDirection(DcMotor.Direction.REVERSE);
            flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            FlywheelController flywheelController = new FlywheelController(flywheel);
            turret.init(hardwareMap, telemetry);

            telemetry.addLine("Center turret BEFORE INIT; encoder zero is the starting position.");
            telemetry.addLine("Sticks: drive | LB: slow | triggers: intake | A/B: transfer forward/reverse");
            telemetry.addLine("D-pad left/right: turret | X: flywheel | RB: auto aim (hold) | Y: stop all");
            telemetry.update();
            waitForStart();
            ElapsedTime loopTime = new ElapsedTime();

            while (opModeIsActive()) {
                double dt = Math.min(loopTime.seconds(), 0.1);
                loopTime.reset();
                if (gamepad1.y) {
                    for (DcMotor motor : motors) motor.setPower(0);
                    turret.enableAutoAim(false);
                    turret.stop();
                    break;
                }

                double scale = gamepad1.left_bumper ? 0.25 : 0.8;
                double[] powers = drivePowers(-gamepad1.left_stick_y,
                        gamepad1.left_stick_x, gamepad1.right_stick_x, scale);
                frontLeft.setPower(powers[0]);
                frontRight.setPower(powers[1]);
                backLeft.setPower(powers[2]);
                backRight.setPower(powers[3]);
                intake.setPower(0.6 * (gamepad1.right_trigger - gamepad1.left_trigger));
                transfer.setPower(gamepad1.a == gamepad1.b ? 0 : gamepad1.a ? 0.8 : -0.8);
                flywheelController.setTargetRPM(gamepad1.x ? FlywheelController.DEFAULT_RPM : 0);

                turret.enableAutoAim(gamepad1.right_bumper);
                if (!gamepad1.right_bumper) {
                    double movement = (gamepad1.dpad_right ? 1 : 0) - (gamepad1.dpad_left ? 1 : 0);
                    turret.setManualTargetAngle(turret.getManualTargetAngle() + movement * 45 * dt);
                }
                turret.update();
                telemetry.addData("Flywheel target RPM", flywheelController.getTargetRPM());
                telemetry.addData("Flywheel measured RPM", flywheelController.getCurrentRPM());
                telemetry.addData("Transfer power", transfer.getPower());
                telemetry.update();
                idle();
            }
        } finally {
            for (DcMotor motor : motors) motor.setPower(0);
            turret.stop();
        }
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
