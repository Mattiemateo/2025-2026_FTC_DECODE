package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "autoball", group = "Vision")
public class autoball extends LinearOpMode {

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive, intake;
    private Limelight3A limelight;

    private static final int    PIPELINE_INDEX = 1;   // matches your redMode pipeline, adjust as needed
    private static final double TARGET_TA      = 9.0; // tune: read ta at your desired stop distance
    private static final double TA_TOLERANCE   = 0.3;
    private static final double TX_TOLERANCE   = 1.0; // degrees

    private static final double MAX_DRIVE = 0.8;
    private static final double MAX_TURN  = 0.5;
    private static final double MIN_DRIVE = 0.08;

    private static final double KP_DRIVE = 0.085;
    private static final double KP_TURN  = 0.02;

    // Flip these to -1.0 if the robot moves the wrong way. Currently set based on
    // observed behavior: robot was driving backwards and turning away -> both flipped.
    private static final double DRIVE_SIGN = -1.0;
    private static final double TURN_SIGN  = -1.0;

    private static final long LOST_TARGET_TIMEOUT_MS = 500;

    @Override
    public void runOpMode() {
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive   = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive  = hardwareMap.get(DcMotor.class, "rightBackDrive");

        intake = hardwareMap.get(DcMotor.class, "intake"); // port 0 Control hub

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotor.Direction.FORWARD);

        for (DcMotor m : new DcMotor[]{leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive}) {
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(PIPELINE_INDEX);
        limelight.start();

        telemetry.addLine("Ready.");
        telemetry.update();
        waitForStart();
        if (isStopRequested()) return;

        long lastSeenTime = System.currentTimeMillis();
        boolean arrived = false;

        while (opModeIsActive() && !arrived) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                lastSeenTime = System.currentTimeMillis();

                double tx = result.getTx();
                double ta = result.getTa();
                double taError = TARGET_TA - ta; // + = too far, need forward power

                if (Math.abs(taError) < TA_TOLERANCE && Math.abs(tx) < TX_TOLERANCE) {
                    arrived = true;
                    intake();
                    stopDrive();
                    break;
                }

                double y = DRIVE_SIGN * clamp(KP_DRIVE * taError, -MAX_DRIVE, MAX_DRIVE);
                if (Math.abs(taError) > TA_TOLERANCE && Math.abs(y) < MIN_DRIVE) {
                    y = Math.copySign(MIN_DRIVE, y);
                }
                double slowFactor = clamp(Math.abs(taError) / TARGET_TA, 0.3, 1.0);
                y *= slowFactor;

                double rx = TURN_SIGN * clamp(KP_TURN * tx, -MAX_TURN, MAX_TURN);
                double x = 0; // no strafing needed, driving straight at target

                double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
                leftFrontDrive.setPower((y + x + rx) / denominator);
                leftBackDrive.setPower((y - x + rx) / denominator);
                rightFrontDrive.setPower((y - x - rx) / denominator);
                rightBackDrive.setPower((y + x - rx) / denominator);

                telemetry.addData("tx", tx);
                telemetry.addData("ta", ta);
                telemetry.addData("taError", taError);
                telemetry.update();

            } else {
                if (System.currentTimeMillis() - lastSeenTime > LOST_TARGET_TIMEOUT_MS) {
                    stopDrive();
                    telemetry.addLine("Target lost - stopped.");
                    telemetry.update();
                }
            }
        }

        stopDrive();
        sleep(500);
    }

    private void stopDrive() {
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }
    private void intake() {
        intake.setPower(-0.7);
        sleep(2000);
        intake.setPower(0);
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}