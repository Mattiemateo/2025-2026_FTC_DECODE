package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@TeleOp
public class reset_cache extends LinearOpMode{

    @Override public void runOpMode() {
        DcMotor rightFrontDrive = hardwareMap.get(DcMotor.class, "rightFrontDrive"); // port 2 Expansion hub
        DcMotor leftFrontDrive = hardwareMap.get(DcMotor.class, "leftFrontDrive"); // port 2 Control hub

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while (opModeIsActive()) {
            leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(300);
        }
    }
}
