package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class LimelightAprilTagDetector {
    private Limelight3A limelight;

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        //telemetry.setMsTransmissionInterval(11);
        limelight.setPollRateHz(100);

        limelight.pipelineSwitch(8);
        /*
         * Starts polling for data.
         */
        limelight.start();
    }
    public Double getTargetBearing() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)
            double dist  = result.getBotposeAvgDist(); // How far away the target is relative to the cam
            return tx;
        } else {
            return null;
        }
    }

    /**
     * Check if the target tag is currently visible
     */
    public boolean isTagVisible() {
        return getTargetBearing() != null;
    }


    /**
    /**
     * Get detailed info about target tag
     */
    /*
    public LimelightAprilTagDetector getTargetDetection() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)
            double dist  = result.getBotposeAvgDist(); // How far away the target is relative to the cam
        } else {
            return null;
        }
    }
    */
    /**
     * Close the vision portal
     */
    public void close() {
        limelight.stop();
    }
}