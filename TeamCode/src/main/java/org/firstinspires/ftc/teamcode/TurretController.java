package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@Configurable
public class TurretController {

    // --- Hardware ---
    private DcMotorEx turretMotor;
    private Limelight3A limelight;
    private IMU imu;
    private Telemetry telemetry;

    // --- Constants ---
    private static final double TICKS_PER_DEGREE =
        ((28 * 20 * 95.0) / 28.0) / 360.0;
    public static double P = 15.0,
        I = 0.0,
        D = 0.0,
        F = 0.0;
    public static double TURRET_MAX_POWER = 0.8;
    public static double TARGET_LOST_TIMEOUT = 2.0;
    public static double SEARCH_SPEED = 0.15; // Slow rotation speed for searching (0.15 = 15% power)

    public static int LIMELIGHT_PIPELINE = 8; // this should probably stay like this

    // --- State ---
    private boolean autoAimEnabled = false;
    private double manualTargetAngle = 0.0;
    private double lastKnownTargetAngleField = 0.0;
    private double lastKnownDistance = 0.0;
    private boolean targetWasVisible = false;
    private final ElapsedTime targetLostTimer = new ElapsedTime();
    private boolean isSearching = false;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            // Initialize the IMU with the modern API (works with both BNO055 and BHI260AP).
            imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot's configuration.
            // This is for a Control Hub that is mounted horizontally with the logo facing up and the USB ports facing forward.
            IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
            );
            imu.initialize(parameters);
            telemetry.addData("IMU", "Initialized successfully");

            turretMotor = hardwareMap.get(DcMotorEx.class, "aim");
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_TO_POSITION,
                new PIDFCoefficients(P, I, D, F)
            );

            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
            limelight.start();

            targetLostTimer.reset();
            telemetry.addData("Turret Controller", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Turret Init Error", e.getMessage());
            telemetry.addData("Stack Trace", e.getClass().getSimpleName());
        }
    }

    public void update() {
        // Update robot orientation for MegaTag 2 (fuses IMU with vision for better accuracy)
        if (imu != null && limelight != null) {
            try {
                double robotYaw = getRobotHeading();
                limelight.updateRobotOrientation(robotYaw);
            } catch (Exception e) {
                // Silently continue if orientation update fails
            }
        }

        if (autoAimEnabled) {
            updateAutoAim();
        } else {
            updateManualControl();
        }
        updateTelemetry();
    }

    private void updateAutoAim() {
        double robotHeading = getRobotHeading();
        double currentTurretAngle = getCurrentAngle();

        LLResult result = limelight.getLatestResult();
        if (hasValidTarget(result)) {
            // TARGET VISIBLE: Track directly using vision
            double tx = result.getFiducialResults().get(0).getTargetXDegrees();

            // Store the field-centric angle for use when target is lost
            lastKnownTargetAngleField = robotHeading + currentTurretAngle + tx;
            targetWasVisible = true;
            targetLostTimer.reset();

            // While target is visible, directly aim at it (simple control)
            double desiredTurretAngle = currentTurretAngle + tx;
            setTargetAngleInternal(desiredTurretAngle);

            // Calculate and store distance to the AprilTag target
            List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();
            if (fiducials != null && !fiducials.isEmpty()) {
                // Get the first detected AprilTag
                LLResultTypes.FiducialResult fiducial = fiducials.get(0);

                // Get robot pose relative to the AprilTag (most useful for distance)
                Pose3D robotPoseTargetSpace =
                    fiducial.getRobotPoseTargetSpace();
                if (robotPoseTargetSpace != null) {
                    // Calculate 3D distance from robot to AprilTag
                    double x = robotPoseTargetSpace.getPosition().x;
                    double y = robotPoseTargetSpace.getPosition().y;
                    double z = robotPoseTargetSpace.getPosition().z;
                    this.lastKnownDistance = Math.sqrt(x * x + y * y + z * z);
                    telemetry.addData(
                        "Distance Method",
                        "RobotPoseTargetSpace"
                    );
                    telemetry.addData(
                        "Distance (X,Y,Z)",
                        "%.1f, %.1f, %.1f",
                        x,
                        y,
                        z
                    );
                } else {
                    telemetry.addData(
                        "Distance Method",
                        "RobotPoseTargetSpace NULL"
                    );
                    telemetry.addData(
                        "Distance Warning",
                        "Enable '3D' in Limelight pipeline"
                    );
                }
            } else {
                telemetry.addData("Distance Method", "No fiducials detected");
            }
        } else if (
            targetWasVisible && targetLostTimer.seconds() < TARGET_LOST_TIMEOUT
        ) {
            // TARGET LOST: Use IMU to maintain field-centric aim
            // Calculate turret angle needed to point at the last known field-centric target
            // This compensates for robot rotation automatically
            double desiredTurretAngle =
                lastKnownTargetAngleField - robotHeading;

            // Normalize angle to [-180, 180] to avoid the turret taking the long way
            while (desiredTurretAngle > 180) desiredTurretAngle -= 360;
            while (desiredTurretAngle < -180) desiredTurretAngle += 360;

            setTargetAngleInternal(desiredTurretAngle);
            isSearching = false;
        } else {
            // TARGET LOST FOR TOO LONG: Start slow 360-degree search
            targetWasVisible = false;
            isSearching = true;

            // Switch to velocity control for continuous rotation
            turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turretMotor.setPower(SEARCH_SPEED); // Slow continuous rotation
        }
    }

    private void updateManualControl() {
        setTargetAngleInternal(manualTargetAngle);
    }

    private void setTargetAngleInternal(double angle) {
        int targetPositionTicks = (int) (angle * TICKS_PER_DEGREE);
        turretMotor.setTargetPosition(targetPositionTicks);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_MAX_POWER);
    }

    public void enableAutoAim(boolean enable) {
        this.autoAimEnabled = enable;
    }

    public void setManualTargetAngle(double angle) {
        this.manualTargetAngle = angle;
    }

    public double getCurrentAngle() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public boolean isTracking() {
        return autoAimEnabled && targetWasVisible;
    }

    public double getLastKnownDistance() {
        return this.lastKnownDistance;
    }

    /**
     * Gets the robot's current heading (yaw) from the IMU.
     * @return The robot's heading in degrees.
     */
    private double getRobotHeading() {
        if (imu == null) {
            telemetry.addData("IMU Error", "IMU not initialized");
            return 0.0;
        }
        try {
            YawPitchRollAngles robotOrientation =
                imu.getRobotYawPitchRollAngles();
            return robotOrientation.getYaw(AngleUnit.DEGREES);
        } catch (Exception e) {
            telemetry.addData("IMU Error", e.getMessage());
            return 0.0;
        }
    }

    private boolean hasValidTarget(LLResult result) {
        if (result == null || !result.isValid()) return false;
        List<LLResultTypes.FiducialResult> fiducials =
            result.getFiducialResults();
        return fiducials != null && !fiducials.isEmpty();
    }

    public void stop() {
        if (turretMotor != null) turretMotor.setPower(0);
        if (limelight != null) limelight.stop();
    }

    public void reset() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        imu.resetYaw(); // Reset the IMU's yaw when resetting the turret
        manualTargetAngle = 0;
    }

    private void updateTelemetry() {
        telemetry.addData(
            "Turret Mode",
            autoAimEnabled ? "Auto-Aim" : "Manual"
        );
        telemetry.addData("Turret Angle", "%.1f°", getCurrentAngle());
        telemetry.addData("Robot Heading", "%.1f°", getRobotHeading());
        if (autoAimEnabled) {
            String status;
            if (isTracking()) {
                status = targetLostTimer.seconds() > 0.1
                    ? "IMU Tracking"
                    : "Vision Locked";
            } else if (isSearching) {
                status = "Searching (360°)";
            } else {
                status = "Searching";
            }
            telemetry.addData("Tracking Status", status);
            telemetry.addData(
                "Target Distance",
                "%.2f inches",
                lastKnownDistance
            );
        }
    }
}
