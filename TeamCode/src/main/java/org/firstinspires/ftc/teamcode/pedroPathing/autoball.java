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

    // --- TUNE THIS ---
    // Point the robot exactly where you want it to stop relative to the ball,
    // read the "ta" telemetry value at that exact spot, and put it here.
    // If the robot is stopping too far away, this number is too LOW - raise it.
    private static final double TARGET_TA      = 9.0;
    private static final double TA_TOLERANCE   = 0.3;
    private static final double TX_TOLERANCE   = 1.0; // degrees

    // Require this many consecutive in-tolerance frames before declaring "arrived".
    // Prevents a single noisy/jittery frame from triggering an early stop.
    private static final int    ARRIVAL_CONFIRM_FRAMES = 5;

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

    private static final double SEARCH_TURN_POWER = 0.25;

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

        // Main loop: keep searching for a ball, approach + intake it, then search again.
        while (opModeIsActive()) {
            boolean found = search();
            if (!found) break; // opmode stopped mid-search

            approachAndIntake();
            // Loop back to search() regardless of whether approach succeeded or the
            // target was lost - search() will just re-find the next ball.
        }

        stopDrive();
        sleep(500);
    }

    /**
     * Rotate in place, scanning, until the Limelight reports a valid target.
     * Returns false only if the opmode was stopped while searching.
     */
    private boolean search() {
        telemetry.addLine("Searching...");
        telemetry.update();

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                stopDrive();
                return true;
            }

            setDrivePowers(0, 0, TURN_SIGN * SEARCH_TURN_POWER);
        }
        return false;
    }

    /**
     * Drive toward the currently tracked target and intake it once arrived.
     * Requires ARRIVAL_CONFIRM_FRAMES consecutive in-tolerance frames before
     * stopping, to avoid stopping early on a single noisy reading.
     * Returns to caller (search loop) if the target is lost or once intake completes.
     */
    private void approachAndIntake() {
        long lastSeenTime = System.currentTimeMillis();
        int consecutiveArrived = 0;

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                lastSeenTime = System.currentTimeMillis();

                double tx = result.getTx();
                double ta = result.getTa();
                double taError = TARGET_TA - ta; // + = too far, need forward power

                boolean inTolerance = Math.abs(taError) < TA_TOLERANCE && Math.abs(tx) < TX_TOLERANCE;

                if (inTolerance) {
                    consecutiveArrived++;
                } else {
                    consecutiveArrived = 0;
                }

                if (consecutiveArrived >= ARRIVAL_CONFIRM_FRAMES) {
                    stopDrive();
                    intake();
                    return;
                }

                double y = DRIVE_SIGN * clamp(KP_DRIVE * taError, -MAX_DRIVE, MAX_DRIVE);
                if (Math.abs(taError) > TA_TOLERANCE && Math.abs(y) < MIN_DRIVE) {
                    y = Math.copySign(MIN_DRIVE, y);
                }
                double slowFactor = clamp(Math.abs(taError) / TARGET_TA, 0.3, 1.0);
                y *= slowFactor;

                double rx = TURN_SIGN * clamp(KP_TURN * tx, -MAX_TURN, MAX_TURN);

                setDrivePowers(y, 0, rx);

                telemetry.addData("tx", tx);
                telemetry.addData("ta", ta);
                telemetry.addData("taError", taError);
                telemetry.addData("consecutiveArrived", consecutiveArrived);
                telemetry.update();

            } else {
                if (System.currentTimeMillis() - lastSeenTime > LOST_TARGET_TIMEOUT_MS) {
                    stopDrive();
                    telemetry.addLine("Target lost - returning to search.");
                    telemetry.update();
                    return; // go back to search()
                }
            }
        }
    }

    private void setDrivePowers(double y, double x, double rx) {
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        leftFrontDrive.setPower((y + x + rx) / denominator);
        leftBackDrive.setPower((y - x + rx) / denominator);
        rightFrontDrive.setPower((y - x - rx) / denominator);
        rightBackDrive.setPower((y + x - rx) / denominator);
    }

    private void stopDrive() {
        setDrivePowers(0, 0, 0);
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