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
    // ta = how much of the camera frame the ball fills. It gets BIGGER the CLOSER
    // the robot gets. The robot drives forward until ta reaches TARGET_TA.
    // If the robot is stopping too far away (not driving far enough forward),
    // TARGET_TA is too LOW - raise it. If it's driving too far / bumping the ball,
    // TARGET_TA is too HIGH - lower it.
    //
    // How to find the right number: run the opmode, manually note the "ta" telemetry
    // value while slowly walking/pushing the robot toward the ball by hand, and read
    // off what ta is at the exact spot you want it to stop. Put that value here.
    private static final double TARGET_TA      = 20.0;
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

    // If the Limelight loses the target after it was already this close (ta this high),
    // assume it just went out of frame under the camera - not a real loss. In that case,
    // drive forward blind for BLIND_DRIVE_MS to close the last gap, then intake.
    // If it's lost before ever getting this close, treat it as a genuine loss and re-search.
    private static final double MIN_TA_FOR_BLIND_APPROACH = 6.0;
    private static final long   BLIND_DRIVE_MS            = 400; // tune: how far to creep forward blind
    private static final double BLIND_DRIVE_POWER         = 0.3;

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
        double lastKnownTa = 0;

        while (opModeIsActive()) {
            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                lastSeenTime = System.currentTimeMillis();

                double tx = result.getTx();
                double ta = result.getTa();
                lastKnownTa = ta;
                double taError = TARGET_TA - ta; // + = too far, need forward power

                // Fallback: if the ball is big enough that it never leaves frame,
                // this still catches "arrived" the old way.
                boolean inTolerance = Math.abs(taError) < TA_TOLERANCE && Math.abs(tx) < TX_TOLERANCE;
                consecutiveArrived = inTolerance ? consecutiveArrived + 1 : 0;

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
                telemetry.update();

            } else {
                // Target not visible this frame.
                if (lastKnownTa >= MIN_TA_FOR_BLIND_APPROACH) {
                    // It was already close - it almost certainly just went under the
                    // camera. Creep forward blind for a fixed burst, then intake.
                    telemetry.addLine("Target lost while close - blind creep + intake.");
                    telemetry.update();
                    blindCreepForward();
                    stopDrive();
                    intake();
                    return;
                }

                if (System.currentTimeMillis() - lastSeenTime > LOST_TARGET_TIMEOUT_MS) {
                    stopDrive();
                    telemetry.addLine("Target lost - returning to search.");
                    telemetry.update();
                    return; // go back to search()
                }
            }
        }
    }

    /** Drive straight forward blind for BLIND_DRIVE_MS to close the final gap once the target is out of frame. */
    private void blindCreepForward() {
        long start = System.currentTimeMillis();
        while (opModeIsActive() && System.currentTimeMillis() - start < BLIND_DRIVE_MS) {
            setDrivePowers(DRIVE_SIGN * BLIND_DRIVE_POWER, 0, 0);
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