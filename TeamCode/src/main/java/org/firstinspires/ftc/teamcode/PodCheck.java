package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * Shows the raw ticks of both odometry pods so you can check them before tuning.
 * Push the robot exactly one tile (24 in.) forward, then one tile left.
 * A goBILDA 4-Bar pod (2000 ticks/rev, 32 mm wheel) reads about 12,100 ticks per 24 in.
 * Press A to zero both readings.
 */
@TeleOp(name = "BIOBUZZ Pod Check", group = "BIOBUZZ")
public class PodCheck extends LinearOpMode {
    public static double EXPECTED_INCHES_PER_TICK = Math.PI * 32.0 / 25.4 / 2000.0;

    @Override
    public void runOpMode() {
        String forwardName = Constants.localizerConfig.xPodName.get();
        String strafeName = Constants.localizerConfig.yPodName.get();
        DcMotorEx forward = hardwareMap.get(DcMotorEx.class, forwardName);
        DcMotorEx strafe = hardwareMap.get(DcMotorEx.class, strafeName);

        telemetry.addLine("Press START, then push the robot by hand. A = zero.");
        telemetry.update();
        waitForStart();

        int forwardZero = forward.getCurrentPosition();
        int strafeZero = strafe.getCurrentPosition();
        while (opModeIsActive()) {
            if (gamepad1.a) {
                forwardZero = forward.getCurrentPosition();
                strafeZero = strafe.getCurrentPosition();
            }
            int f = forward.getCurrentPosition() - forwardZero;
            int s = strafe.getCurrentPosition() - strafeZero;
            telemetry.addData("Forward pod (" + forwardName + ")", "%d ticks = %.1f in", f, f * EXPECTED_INCHES_PER_TICK);
            telemetry.addData("Strafe pod (" + strafeName + ")", "%d ticks = %.1f in", s, s * EXPECTED_INCHES_PER_TICK);
            telemetry.addLine("Push forward: only the forward pod should change.");
            telemetry.addLine("Push left: only the strafe pod should change.");
            telemetry.update();
        }
    }
}
