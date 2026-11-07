package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.follower.Follower;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.Encoder;
import com.pedropathing.revhub.localizers.RevHubIMU;
import com.pedropathing.revhub.localizers.TwoWheelConfig;
import com.pedropathing.revhub.localizers.TwoWheelLocalizer;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("leftFrontDrive");
        c.frontRightName.set("rightFrontDrive");
        c.backLeftName.set("leftBackDrive");
        c.backRightName.set("rightBackDrive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // These are encoder-port names, even when pods share ports with drive motors.
    public static TwoWheelConfig localizerConfig = new TwoWheelConfig(c -> {
        c.xPodName.set("rightBackDrive");
        c.yPodName.set("rightFrontDrive");
        c.imuName.set("imu");
        // Two Wheel Tuner output, 2026-11-07 (second run, forward pod fixed and moved to rightBackDrive).
        // NOTE: strafeTicksToInches 0.00357 is ~1.8x the first run (0.00203) and the ~0.00198 expected
        // for a goBILDA 4-Bar pod. Check one tile sideways with "BIOBUZZ Pod Check" / Robot Pose.
        c.xPodOffset.set(-2.9544544296830106);
        c.yPodOffset.set(-0.31120939571087114);
        c.forwardTicksToInches.set(0.0020144555737033826);
        c.strafeTicksToInches.set(0.003571875173051589);
        c.xPodDirection.set(Encoder.REVERSE);
        c.yPodDirection.set(Encoder.FORWARD);
        c.imu.set(new RevHubIMU(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP)));
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        // Paste Foresight Tuner output here after tuning the localizer.
    });

    public static Mecanum createDrivetrain(HardwareMap hardwareMap) {
        return new Mecanum(hardwareMap, drivetrainConfig);
    }

    public static TwoWheelLocalizer createLocalizer(HardwareMap hardwareMap) {
        return new TwoWheelLocalizer(hardwareMap, localizerConfig);
    }

    public static Foresight createAlgorithm() {
        return new Foresight(foresightConfig);
    }

    public static Follower create(HardwareMap hardwareMap) {
        return new Follower(createLocalizer(hardwareMap), createDrivetrain(hardwareMap), createAlgorithm());
    }
}
