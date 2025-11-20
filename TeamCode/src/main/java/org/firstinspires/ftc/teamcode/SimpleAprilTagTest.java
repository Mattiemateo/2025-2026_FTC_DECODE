package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Simple AprilTag Test")
@Disabled
public class SimpleAprilTagTest extends LinearOpMode {

    private AprilTagDetector aprilTagDetector;

    @Override
    public void runOpMode() {
        // Initialize AprilTag detector
        aprilTagDetector = new AprilTagDetector();
        aprilTagDetector.init(hardwareMap);
        aprilTagDetector.setTargetTagId(20); // Looking for tag ID 1

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Looking for", "Tag ID 20");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get all detections
            List<AprilTagDetection> detections = aprilTagDetector.getAllDetections();

            telemetry.addData("Total Tags Detected", detections.size());

            // Show info for each detected tag
            for (AprilTagDetection detection : detections) {
                telemetry.addData("Tag ID", detection.id);

                if (detection.ftcPose != null) {
                    telemetry.addData("  Bearing", "%.1f°", detection.ftcPose.bearing);
                    telemetry.addData("  Range", "%.1f centimeters", detection.ftcPose.range);
                    telemetry.addData("  Yaw", "%.1f°", detection.ftcPose.yaw);
                } else {
                    telemetry.addData("  Pose", "NULL - still processing");
                }
                telemetry.addLine("---");
            }

            // Test our helper methods
            telemetry.addLine("Helper Methods:");
            telemetry.addData("Target Tag Visible", aprilTagDetector.isTagVisible() ? "YES" : "NO");

            Double bearing = aprilTagDetector.getTargetBearing();
            if (bearing != null) {
                telemetry.addData("Target Bearing", "%.1f°", bearing);
            }

            Double distance = aprilTagDetector.getTargetDistance();
            if (distance != null) {
                telemetry.addData("Target Distance", "%.1f centimeters", distance);
            }

            telemetry.update();
            sleep(100); // Slow down updates a bit
        }

        aprilTagDetector.close();
    }
}