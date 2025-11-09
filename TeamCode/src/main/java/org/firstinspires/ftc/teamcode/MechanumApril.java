package org.firstinspires.ftc.teamcode;
//import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class MechanumApril extends LinearOpMode {
    private DcMotor flywheel;
    private DcMotor aim;
    private Servo hood;
    private DcMotor intake;
    private DcMotor leftFrontDrive;
    private DcMotor rightFrontDrive;
    private DcMotor leftBackDrive;
    private DcMotor rightBackDrive;

    int aimpos = 0;
    double hoodpos = 0;
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    //private TelemetryManager telemetryM;

    @Override
    public void runOpMode() {
        // Initialize motors and servos
        leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive");
        leftBackDrive = hardwareMap.get(DcMotor.class, "leftBackDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightBackDrive");

        intake = hardwareMap.get(DcMotor.class, "intake");
        hood = hardwareMap.get(Servo.class, "hood");
        flywheel = hardwareMap.get(DcMotor.class, "flywheel");
        aim = hardwareMap.get(DcMotor.class, "aim");


        leftFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        aim.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //aim.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Motor directions
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        intake.setDirection(DcMotor.Direction.FORWARD);
        flywheel.setDirection(DcMotor.Direction.REVERSE);
        aim.setDirection(DcMotor.Direction.FORWARD);

        aim.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        hood.setPosition(0.0);

        initAprilTag();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            aim.setTargetPosition(aimpos);

            telemetryAprilTag();

            // Driving
            double x = -gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double theta = Math.atan2(y, x);
            double power = Math.hypot(x, y);
            double sin = Math.sin(theta - Math.PI / 4);
            double cos = Math.cos(theta - Math.PI / 4);
            double leftFront = power * cos + turn;
            double rightFront = power * sin - turn;
            double leftBack = power * sin + turn;
            double rightBack = power * cos - turn;

            if ((power + Math.abs(turn)) > 1) {
                leftFront /= power + Math.abs(turn);
                leftBack /= power + Math.abs(turn);
                rightFront /= power + Math.abs(turn);
                rightBack /= power + Math.abs(turn);
            }

            double scale = gamepad1.left_trigger > 0.5 ? 0.25 : 1.0;

            leftFrontDrive.setPower(leftFront * scale);
            leftBackDrive.setPower(leftBack * scale);
            rightFrontDrive.setPower(rightFront * scale);
            rightBackDrive.setPower(rightBack * scale);


            if (gamepad1.circle) {
                intake.setPower(1);
            }else{
                intake.setPower(0);
            }

            if (gamepad1.cross) {
                flywheel.setPower(1);
            }else{
                flywheel.setPower(0);
            }

            //if (!gamepad1.cross) {
            //    if (gamepad1.dpad_left) {
            //        aimpos = Math.max(0, Math.min(aimpos+5, 1259));
            //        aim.setTargetPosition(aimpos);
            //        aim.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //    }else if (gamepad1.dpad_right) {
            //        aimpos += Math.max(0, Math.min(aimpos-5, 1259));
            //        aim.setTargetPosition(aimpos);
            //        aim.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //    }
            //}

            if (gamepad1.dpad_up) {
                hoodpos = Math.max(0, Math.min(hoodpos+0.05, 1));
                hood.setPosition(hoodpos);
            }else if (gamepad1.dpad_down){
                hoodpos = Math.max(0, Math.min(hoodpos-0.05, 1));
                hood.setPosition(hoodpos);
            }

            aim.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            aim.setPower(0.5);

            telemetry.addData("Aimpos", aimpos);
            telemetry.addData("hoodpos", hoodpos);

            telemetry.update();
            sleep(20);
        }

        visionPortal.close();
    }

    private void initAprilTag() {
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();

        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, aprilTag);
        }

    }

    private void telemetryAprilTag() {
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
            aimpos += detection.ftcPose.x;
            aim.setTargetPosition(aimpos);
            aim.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
    }

}