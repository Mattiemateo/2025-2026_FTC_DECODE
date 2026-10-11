package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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
            .mass(8)//in KG
            .forwardZeroPowerAcceleration(-53)
            .lateralZeroPowerAcceleration(-75)//tdo -67 -81 -78 -71
            //.translationalPIDFCoefficients(new PIDFCoefficients(0.08, 0, 0.01, 0)) //TDO test panels translation
            //.drivePIDFCoefficients(new FilteredPIDFCoefficients(0.003, 0.0, 0.0005, 0.6, 0.0)) //TDO test panels drive











            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.03, 0.02)) //TDO test panels heading (0.8, 0, 0.1, 0.03)













            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.05, 0.077, 0.0021)) // (kP, kLinear, kQuadratic) (0.05, 0.077, 0.0021)
            .centripetalScaling(0); //TDO test panels centripetal
    public static PathConstraints pathConstraints = new PathConstraints(0.8, 20, Math.toRadians(180), Math.toRadians(360));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFrontDrive")
            .rightRearMotorName("rightBackDrive")
            .leftRearMotorName("leftBackDrive")
            .leftFrontMotorName("leftFrontDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            .xVelocity(60.4) //TOO test forwardvelocitytuner
            .yVelocity(49.0); //TDO test lateralvelocitytuner

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("leftBackDrive")
            .strafeEncoder_HardwareMapName("rightFrontDrive")
            .forwardPodY(-7.5)//TDO: measure offset from center to pod in inches7.5
            .strafePodX(0.5)//TDO: measure offset from center to pod in inches-0.5

            .forwardEncoderDirection(Encoder.REVERSE)//TDO
            //.strafeEncoderDirection(Encoder.REVERSE)//TDO

            .forwardTicksToInches(0.001979)//TDO: Tune
            .strafeTicksToInches(0.001979)//TDO: Tune

            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, //TDO
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
