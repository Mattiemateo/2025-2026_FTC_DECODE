package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class AprilTagDetector {
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    // Target tag ID (change this to the tag you want to track)
    private int targetTagId = 20;

    /**
     * Initialize the AprilTag detector with Logitech C270
     */
    public void init(HardwareMap hardwareMap) {
        // Create the AprilTag processor with specified settings
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                //.setDecimation(2)  // Process every 2nd pixel (2 or 3 for speed)
                .setNumThreads(3)  // Use multiple CPU threads
                .build();

        // Create the vision portal with the webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .setCameraResolution(new android.util.Size(640, 480))  // Or 320x240 for max speed
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(false)
                //.setAutoStopLiveView(true)
                .build();
    }

    /**
     * Get the bearing angle to the target AprilTag
     * Returns null if tag is not detected or metadata not available
     */
    public Double getTargetBearing() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {
            if (detection.id == targetTagId && detection.metadata != null && detection.ftcPose != null) {
                return detection.ftcPose.bearing;
            }
        }

        return null;
    }

    /**
     * Check if the target tag is currently visible
     */
    public boolean isTagVisible() {
        return getTargetBearing() != null;
    }

    /**
     * Get count of all detected tags (for debugging)
     */
    public int getDetectionCount() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        return detections != null ? detections.size() : 0;
    }

    /**
     * Get the distance to the target tag in centimeters
     */
    public Double getTargetDistance() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {
            if (detection.id == targetTagId && detection.metadata != null && detection.ftcPose != null) {
                return detection.ftcPose.range;
            }
        }

        return null;
    }

    /**
     * Set which AprilTag ID to track
     */
    public void setTargetTagId(int id) {
        targetTagId = id;
    }

    /**
     * Get current target tag ID
     */
    public int getTargetTagId() {
        return targetTagId;
    }

    /**
     * Get all detected tags (for debugging)
     */
    public List<AprilTagDetection> getAllDetections() {
        return aprilTag.getDetections();
    }

    /**
     * Get detailed info about target tag
     */
    public AprilTagDetection getTargetDetection() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection detection : detections) {
            if (detection.id == targetTagId) {
                return detection;
            }
        }

        return null;
    }

    /**
     * Close the vision portal
     */
    public void close() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}