package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.ToDoubleBiFunction;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9)
            .forwardZeroPowerAcceleration(-44) //TODO test forwardzeropoweraccelerationtuner -45, -43 -
            .lateralZeroPowerAcceleration(-72) //TODO test lateralzeropoweraccelerationtuner -73
            .translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.01, 0)) //TODO test panels translation
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0, 0.01, 0)) //TODO test panels heading
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.003,0.0,0.005,0.6,0.0)) //TODO test panels drive
            .centripetalScaling(0.005); //TODO test panels centripetal
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFrontDrive")
            .rightRearMotorName("rightBackDrive")
            .leftRearMotorName("leftBackDrive")
            .leftFrontMotorName("leftFrontDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)

            .xVelocity(64.8) //TODO test forwardvelocitytuner
            .yVelocity(42.1); //TODO test lateralvelocitytuner

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("leftFrontDrive")
            .strafeEncoder_HardwareMapName("rightBackDrive")
            .forwardPodY(7.7)//TDO: measure offset from center to pod in inches
            .strafePodX(-6.5)//TDO: measure offset from center to pod in inches

            //.forwardEncoderDirection(Encoder.REVERSE)//TDO
            //.strafeEncoderDirection(Encoder.REVERSE)//TDO

            .forwardTicksToInches(0.002046303932)//TDO: Tune
            .strafeTicksToInches(0.001723990049)//TDO: Tune

            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.FORWARD, //TDO
                            RevHubOrientationOnRobot.UsbFacingDirection.UP //TDO
                    )
            );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }
}
