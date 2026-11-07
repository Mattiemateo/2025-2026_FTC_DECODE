package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class TurretController {

    // --- Hardware ---
    private DcMotorEx turretMotor;
    private Limelight3A limelight;
    private IMU imu;
    private Telemetry telemetry;

    // --- Constants ---
    private static final double TICKS_PER_DEGREE =
        ((28 * 20 * 95.0) / 28.0) / 360.0;
    // Hub RUN_TO_POSITION gain (SDK default is 5). Higher = stiffer hold, but jerky/overshoot.
    public static double POSITION_P = 5.0;
    // Hub velocity loop used by the bumpers (RUN_USING_ENCODER) and inside RUN_TO_POSITION.
    // Leave USE_CUSTOM_VELOCITY_PIDF false to keep the hub's defaults for the configured motor.
    public static boolean USE_CUSTOM_VELOCITY_PIDF = false;
    public static double VELOCITY_P = 10.0, VELOCITY_I = 3.0, VELOCITY_D = 0.0, VELOCITY_F = 0.0;
    // shortcut: provisional travel limits, measure safe travel before widening them.
    public static double MIN_ANGLE = -45.0, MAX_ANGLE = 45.0;
    public static double TURRET_MAX_POWER = 0.3;
    // Power used while a turret button is held; on release the turret holds where it stopped.
    public static double MANUAL_SPEED_DEG_PER_SEC = 60.0;
    public static double SEARCH_POWER = 0.3; // Slower power for search movements
    public static double SEARCH_DWELL_TIME = 0.3; // Time to wait at each position (seconds)
    public static double TARGET_LOST_TIMEOUT = 2.0;
    public static double SEARCH_INITIAL_ANGLE = 15.0; // Initial search sweep angle
    public static double SEARCH_ANGLE_INCREMENT = 20.0; // Angle increment per sweep
    public static double SEARCH_MAX_ANGLE = 135.0; // Maximum search sweep angle
    public static boolean IMU_SEARCH_ENABLED = true; // Enable/disable IMU-based search when target is lost
    public int LIMELIGHT_PIPELINE = 8; // this should probably stay like this

    // --- State ---
    private boolean autoAimEnabled = false;
    private double manualTargetAngle = 0.0;
    private double manualDirection = 0.0;
    private double lastKnownTargetAngleField = 0.0;
    private double lastKnownDistance = 0.0;
    private boolean targetWasVisible = false;
    private final ElapsedTime targetLostTimer = new ElapsedTime();
    private boolean isSearching = false;
    private double searchAngle = SEARCH_INITIAL_ANGLE; // Current search sweep angle
    private double searchCenterAngle = 0.0; // Center point of the search
    private boolean searchingRight = true; // Direction of current search sweep
    private final ElapsedTime searchDwellTimer = new ElapsedTime(); // Timer for dwelling at each position
    private boolean isDwelling = false; // Whether we're currently dwelling at a position

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        try {
            // Initialize the IMU with the modern API (works with both BNO055 and BHI260AP).
            imu = hardwareMap.get(IMU.class, "imu");
            // Adjust the orientation parameters to match your robot's configuration.
            // This is for a Control Hub that is mounted horizontally with the logo facing up and the USB ports facing forward.
            IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                    RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                    RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
            );
            imu.initialize(parameters);
            telemetry.addData("IMU", "Initialized successfully");

            turretMotor = hardwareMap.get(DcMotorEx.class, "turret"); // port 0 Expansion hub
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turretMotor.setPositionPIDFCoefficients(POSITION_P);
            if (USE_CUSTOM_VELOCITY_PIDF) {
                turretMotor.setVelocityPIDFCoefficients(VELOCITY_P, VELOCITY_I, VELOCITY_D, VELOCITY_F);
            }

            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
            limelight.start();

            targetLostTimer.reset();
            telemetry.addData("Turret Controller", "Initialized");
        } catch (Exception e) {
            telemetry.addData("Turret Init Error", e.getMessage());
            telemetry.addData("Stack Trace", e.getClass().getSimpleName());
            stop();
            throw new IllegalStateException("Turret initialization failed", e);
        }
    }

    public void update() {
        // Update robot orientation for MegaTag 2 (fuses IMU with vision for better accuracy)
        if (limelight == null || turretMotor == null) {
            telemetry.addData("Turret", "Not initialized, skipping update");
            return;
        }
        if (imu != null) {
            try {
                double robotYaw = getRobotHeading();
                limelight.updateRobotOrientation(robotYaw);
            } catch (Exception e) {
                // Silently continue if orientation update fails
                telemetry.addData("Turret Init Error", e.getMessage());
                telemetry.update();
                throw new RuntimeException("Turret init failed: " + e.getMessage(), e);
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

            // If we were searching, stop and transition back to position control
            if (isSearching) {
                turretMotor.setPower(0); // Stop the search rotation
                isSearching = false;
                // Reset search angle for next time
                searchAngle = SEARCH_INITIAL_ANGLE;
            }

            double tx = result.getFiducialResults().get(0).getTargetXDegrees();

            // Store the field-centric angle for use when target is lost
            // Note: Negating robotHeading to fix reversed IMU compensation
            lastKnownTargetAngleField = -robotHeading + currentTurretAngle + tx;
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
            targetWasVisible &&
            targetLostTimer.seconds() < TARGET_LOST_TIMEOUT &&
            IMU_SEARCH_ENABLED
        ) {
            // TARGET LOST: Use IMU-based search to maintain field-centric aim (if enabled)
            // Calculate turret angle needed to point at the last known field-centric target
            // This compensates for robot rotation automatically
            // Note: Negating robotHeading to fix reversed IMU compensation
            double desiredTurretAngle =
                lastKnownTargetAngleField - (-robotHeading);

            // Normalize angle to [-180, 180] to avoid the turret taking the long way
            while (desiredTurretAngle > 180) desiredTurretAngle -= 360;
            while (desiredTurretAngle < -180) desiredTurretAngle += 360;

            setTargetAngleInternal(desiredTurretAngle);
            isSearching = false;
        } else {
            // TARGET LOST: IMU tracking disabled or timeout expired
            // Start oscillating search pattern
            targetWasVisible = false;

            if (!isSearching) {
                // Just started searching - record the center position
                isSearching = true;
                searchCenterAngle = currentTurretAngle;
                searchAngle = SEARCH_INITIAL_ANGLE;
                searchingRight = true;
                isDwelling = false;
            }

            // Check if we're dwelling (waiting at a position)
            if (isDwelling) {
                // Wait for dwell time before moving to next position
                if (searchDwellTimer.seconds() >= SEARCH_DWELL_TIME) {
                    isDwelling = false; // Done dwelling, move to next position
                } else {
                    // Still dwelling, don't move yet
                    return;
                }
            }

            // Check if we've reached the current search limit
            double currentAngle = getCurrentAngle();
            double targetAngle;
            boolean shouldMove = false;

            if (searchingRight) {
                targetAngle = clampAngle(searchCenterAngle + searchAngle);
                // If we've reached or passed the right limit, switch direction
                if (currentAngle >= targetAngle - 2) {
                    // 2 degree tolerance
                    searchingRight = false;
                    isDwelling = true;
                    searchDwellTimer.reset();
                } else {
                    shouldMove = true;
                }
            } else {
                targetAngle = clampAngle(searchCenterAngle - searchAngle);
                // If we've reached or passed the left limit, switch direction and increase angle
                if (currentAngle <= targetAngle + 2) {
                    // 2 degree tolerance
                    searchingRight = true;
                    isDwelling = true;
                    searchDwellTimer.reset();
                    // Increase search angle for next sweep, up to maximum
                    searchAngle = Math.min(
                        searchAngle + SEARCH_ANGLE_INCREMENT,
                        SEARCH_MAX_ANGLE
                    );
                } else {
                    shouldMove = true;
                }
            }

            if (shouldMove) {
                // Move to target position with reduced power
                int targetPositionTicks = (int) (targetAngle *
                    TICKS_PER_DEGREE);
                turretMotor.setTargetPosition(targetPositionTicks);
                setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turretMotor.setPower(SEARCH_POWER); // Use slower search power
            }
        }
    }

    private void updateManualControl() {
        double degPerSec = manualVelocity(manualDirection, getCurrentAngle());
        if (degPerSec != 0) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            turretMotor.setVelocity(degPerSec * TICKS_PER_DEGREE);
            manualTargetAngle = clampAngle(getCurrentAngle());
        } else {
            setTargetAngleInternal(manualTargetAngle);
        }
    }

    /** Jog speed in deg/s, or 0 when not jogging or when the travel limit is reached. */
    static double manualVelocity(double direction, double currentAngle) {
        if (direction > 0 && currentAngle < MAX_ANGLE) return MANUAL_SPEED_DEG_PER_SEC;
        if (direction < 0 && currentAngle > MIN_ANGLE) return -MANUAL_SPEED_DEG_PER_SEC;
        return 0;
    }

    /** Only send a mode change to the hub when the mode actually changes. */
    private void setMode(DcMotor.RunMode mode) {
        if (turretMotor.getMode() != mode) turretMotor.setMode(mode);
    }

    private void setTargetAngleInternal(double angle) {
        int targetPositionTicks = (int) (clampAngle(angle) * TICKS_PER_DEGREE);
        turretMotor.setTargetPosition(targetPositionTicks);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_MAX_POWER);
    }

    public void enableAutoAim(boolean enable) {
        if (autoAimEnabled && !enable) {
            manualTargetAngle = getCurrentAngle();
            isSearching = false;
        }
        this.autoAimEnabled = enable;
    }

    public void setManualTargetAngle(double angle) {
        this.manualTargetAngle = clampAngle(angle);
    }

    /** +1 turns the turret toward MAX_ANGLE, -1 toward MIN_ANGLE, 0 holds position. Applied in update(). */
    public void jogManual(double direction) {
        manualDirection = Math.signum(direction);
    }

    public double getManualTargetAngle() {
        return manualTargetAngle;
    }

    static double clampAngle(double angle) {
        if (!Double.isFinite(angle) || !Double.isFinite(MIN_ANGLE)
                || !Double.isFinite(MAX_ANGLE) || MIN_ANGLE >= MAX_ANGLE) {
            throw new IllegalArgumentException("Turret angle and travel limits must be finite and ordered");
        }
        return Math.max(MIN_ANGLE, Math.min(MAX_ANGLE, angle));
    }

    public double getCurrentAngle() {
        return turretMotor.getCurrentPosition() / TICKS_PER_DEGREE;
    }

    public void setPipeline(int pipeline) {
        LIMELIGHT_PIPELINE = pipeline;
        if (limelight != null) {
            limelight.pipelineSwitch(pipeline);
        }
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
        telemetry.addData("Turret Ticks (target/current)", "%d / %d",
            turretMotor.getTargetPosition(), turretMotor.getCurrentPosition());
        telemetry.addData("Turret Power / Velocity", "%.2f / %.0f t/s",
            turretMotor.getPower(), turretMotor.getVelocity());
        telemetry.addData("Turret Jog Dir / Run Mode", "%.0f / %s",
            manualDirection, turretMotor.getMode());
        telemetry.addData("Robot Heading", "%.1f°", getRobotHeading());
        if (autoAimEnabled) {
            String status;
            if (isTracking()) {
                status = targetLostTimer.seconds() > 0.1
                    ? "IMU Tracking"
                    : "Vision Locked";
            } else if (isSearching) {
                status = "Searching (limited travel)";
            } else {
                status = "Searching";
            }
            telemetry.addData("Tracking Status", status);
        }
    }

}
