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
            .mass(5)//TODO: robot weight in KGs
            .forwardZeroPowerAcceleration(0) //TODO test forwardzeropoweraccelerationtuner
            .lateralZeroPowerAcceleration(0) //TODO test lateralzeropoweraccelerationtuner
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0)) //TODO test panels translation
            .headingPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0)) //TODO test panels heading
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1,0.0,0.01,0.6,0.0)) //TODO test panels drive
            .centripetalScaling(0.005); //TODO test panels centripetal
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFrontDrive")
            .rightRearMotorName("rightBackDrive")
            .leftRearMotorName("leftBackDrive")
            .leftFrontMotorName("leftFrontDrive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)

            .xVelocity(0) //TODO test forwardvelocitytuner
            .yVelocity(0); //TODO test lateralvelocitytuner

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("leftFront")//TODO: Make sure to replace the hardware map names with the actual names of the motor port they are plugged into
            .strafeEncoder_HardwareMapName("rightRear")//TODO: Make sure to replace the hardware map names with the actual names of the motor port they are plugged into

            .forwardPodY(3)//TODO: measure offset from center to pod in inches
            .strafePodX(3)//TODO: measure offset from center to pod in inches

            .forwardEncoderDirection(Encoder.REVERSE)//TODO
            .strafeEncoderDirection(Encoder.REVERSE)//TODO

            .forwardTicksToInches(0)//TODO: Tune
            .strafeTicksToInches(0)//TODO: Tune

            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP, //TODO
                            RevHubOrientationOnRobot.UsbFacingDirection.LEFT //TODO
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
