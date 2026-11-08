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
    // Travel limits, 0 = straight ahead, clockwise positive. The cables allow 180 deg CCW and
    // 120 deg CW; these keep a 10 deg margin. Right side of the robot = +90, left side = -90.
    public static double MIN_ANGLE = -170.0, MAX_ANGLE = 110.0;
    public static double TURRET_MAX_POWER = 0.3;
    // Power used while a turret button is held; on release the turret holds where it stopped.
    public static double MANUAL_SPEED_DEG_PER_SEC = 60.0;
    public static double SEARCH_POWER = 0.3; // Slower power for search movements
    public static double SEARCH_DWELL_TIME = 0.3; // Time to wait at each position (seconds)
    public static double TARGET_LOST_TIMEOUT = 2.0;
    public static double SEARCH_INITIAL_ANGLE = 15.0; // Initial search sweep angle
    public static double SEARCH_ANGLE_INCREMENT = 20.0; // Angle increment per sweep
    public static double SEARCH_MAX_ANGLE = 135.0; // Maximum search sweep angle
    // --- Where auto-aim gets the goal direction from ---
    // ODOMETRY: robot pose from the odometry pods + IMU (see FieldAim). LIMELIGHT: camera tags.
    public enum AimSource { ODOMETRY, LIMELIGHT }
    public static AimSource AIM_SOURCE = AimSource.LIMELIGHT;

    // --- BIOBUZZ goal tags (Competition Manual TU08, section 9.9) ---
    // Each CELL has a cluster of four 3.25 in. 36h11 tags on its underside: red 30-33 / 34-37,
    // blue 38-41 / 42-45. Set the Limelight pipeline's tag size to 82.55 mm and enable full 3D.
    public enum Alliance { RED, BLUE, ANY }
    public static Alliance ALLIANCE = Alliance.ANY;
    static final int FIRST_TAG_ID = 30;
    // Tag centres along the sticker, relative to the cluster centre (6.5 in. and 2.75 in.).
    static final double[] TAG_OFFSETS_M = {-0.1651, -0.06985, 0.06985, 0.1651};

    // --- Gyro-stabilised auto-aim ---
    // Set from the bumpers: true if RIGHT bumper (positive angle) turns the turret clockwise seen from above.
    // Wrong value = the turret turns WITH the robot instead of against it, and aims away from the tag.
    public static boolean TURRET_POSITIVE_IS_CLOCKWISE = true;
    public static double VISION_FILTER = 0.5; // 0..1, how much each camera frame corrects the aim (lower = smoother)
    public static double GYRO_LEAD_SEC = 0.05; // aim ahead by turn rate x this, covers motor lag (0 = off)
    public static double AIM_MAX_POWER = 0.6; // auto-aim needs to be faster than manual to keep up with spins
    public static boolean IMU_SEARCH_ENABLED = true;

    // --- Shooting while moving ---
    // Limelight mounting (measured 2026-11-08): lens 25 cm above the floor, optical axis 55 deg
    // from VERTICAL, i.e. tilted up 35 deg above level. This is the angle above level.
    // The tilt turns the camera's 3D tag position into horizontal distance and direction.
    // Check: "Goal height" in telemetry should match the real tag height at every distance;
    // if it drifts with distance, adjust CAMERA_PITCH_DEG.
    public static double CAMERA_PITCH_DEG = 35.0;
    public static double CAMERA_HEIGHT_IN = 25.0 / 2.54;
    // Scales the shot lead for robot motion: >1 if moving shots land behind the goal, <1 if ahead.
    public static double LEAD_GAIN = 1.0; // Enable/disable IMU-based search when target is lost
    public int LIMELIGHT_PIPELINE = 1; // this should probably stay like this

    // --- State ---
    private boolean autoAimEnabled = false;
    private double manualTargetAngle = 0.0;
    private double manualDirection = 0.0;
    private double targetBearingField = 0.0; // target direction in field frame (turret degrees)
    private double aimTargetAngle = 0.0;
    private ClusterFix lastClusterFix = null;
    private double robotVForward = 0.0; // in/s, robot frame
    private double robotVLeft = 0.0; // in/s, robot frame
    private double goalDistanceIn = 0.0; // horizontal, from the last fix with 3D data
    private double shotDistanceIn = 0.0; // horizontal distance to the lead-compensated aim point
    private double leadOffsetDeg = 0.0;
    private long lastFixNanos = -1;
    private boolean fieldAimValid = false;
    private double fieldAimAngle = 0.0;
    private long lastFrameNanos = -1;
    private static final int HISTORY_SIZE = 50;
    private final long[] historyNanos = new long[HISTORY_SIZE];
    private final double[] historyHeading = new double[HISTORY_SIZE];
    private final double[] historyTurret = new double[HISTORY_SIZE];
    private int historyIndex = 0;
    private int historyCount = 0;
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

            if (AIM_SOURCE == AimSource.LIMELIGHT) {
                limelight = hardwareMap.get(Limelight3A.class, "limelight");
                limelight.pipelineSwitch(LIMELIGHT_PIPELINE);
                limelight.start();
            }

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
        if (turretMotor == null) {
            telemetry.addData("Turret", "Not initialized, skipping update");
            return;
        }
        if (imu != null && limelight != null) {
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

        recordHistory(getRobotHeading(), getCurrentAngle());
        if (autoAimEnabled && AIM_SOURCE == AimSource.ODOMETRY) {
            if (fieldAimValid) {
                aimTargetAngle = fieldAimAngle;
                driveToAngle(fieldAimAngle, AIM_MAX_POWER);
            } else {
                setTargetAngleInternal(manualTargetAngle); // no pose yet: hold position
            }
        } else if (autoAimEnabled && limelight != null) {
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
        ClusterFix fix = findCluster(result);
        if (fix != null) {
            // TARGET VISIBLE: update the target's field bearing from this camera frame.
            if (isSearching) {
                isSearching = false;
                searchAngle = SEARCH_INITIAL_ANGLE;
            }
            long frameNanos = result.getControlHubTimeStampNanos();
            if (frameNanos != lastFrameNanos) {
                lastFrameNanos = frameNanos;
                // Use the heading and turret angle from when the frame was captured, not now.
                double ageMs = result.getStaleness() + result.getCaptureLatency() + result.getTargetingLatency();
                double[] then = historyAt(System.nanoTime() - (long) (ageMs * 1e6));
                double tx = fix.txDeg;
                double measured = fieldBearing(then[1], tx, then[0]);
                targetBearingField = targetWasVisible
                    ? wrap180(targetBearingField + VISION_FILTER * wrap180(measured - targetBearingField))
                    : measured;
            }
            targetWasVisible = true;
            targetLostTimer.reset();
            aimAtBearing(robotHeading);
            lastClusterFix = fix;
            if (fix.distanceM > 0) {
                lastKnownDistance = fix.distanceM;
                goalDistanceIn = fix.horizontalIn;
                lastFixNanos = System.nanoTime();
            }
        } else if (
            targetWasVisible &&
            targetLostTimer.seconds() < TARGET_LOST_TIMEOUT &&
            IMU_SEARCH_ENABLED
        ) {
            // TARGET LOST: keep pointing at the last known field bearing using the IMU only.
            aimAtBearing(robotHeading);
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

    /**
     * Odometry aim: point the turret at a field point from the robot's pose.
     * Field frame: inches, heading in degrees counter-clockwise (see FieldAim).
     * Call every loop before update(); the turn-rate lead covers motor lag.
     */
    public void setFieldAim(double robotX, double robotY, double robotHeadingDeg, double goalX, double goalY) {
        double leadHeading = robotHeadingDeg + getYawRateDegPerSec() * GYRO_LEAD_SEC;
        fieldAimAngle = turretAngleToPoint(robotX, robotY, leadHeading, goalX, goalY);
        fieldAimValid = true;
    }

    /** Stop odometry aim (e.g. pose unknown); the turret then holds where it is. */
    public void clearFieldAim() {
        fieldAimValid = false;
    }

    /** Turret angle that points from the robot at (goalX, goalY). */
    static double turretAngleToPoint(double robotX, double robotY, double robotHeadingDeg,
                                     double goalX, double goalY) {
        double bearing = Math.toDegrees(Math.atan2(goalY - robotY, goalX - robotX));
        return wrap180(turretSign() * (robotHeadingDeg - bearing));
    }

    private void driveToAngle(double angle, double maxPower) {
        int targetPositionTicks = (int) (reachableAngle(angle, getCurrentAngle()) * TICKS_PER_DEGREE);
        turretMotor.setTargetPosition(targetPositionTicks);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(maxPower);
    }

    /** Gyro-stabilised aim: counter-rotate the turret against the robot's heading (and turn rate). */
    private void aimAtBearing(double robotHeading) {
        double leadHeading = robotHeading + getYawRateDegPerSec() * GYRO_LEAD_SEC;
        double goalAngle = turretAngleFor(targetBearingField, leadHeading);
        if (goalDistanceIn > 0) {
            // Aim where the ball will meet the goal, given the robot's own motion.
            double[] lead = leadSolution(goalAngle, goalDistanceIn, robotVForward, robotVLeft,
                LEAD_GAIN, ShotTable.ROWS);
            aimTargetAngle = lead[0];
            shotDistanceIn = lead[1];
        } else {
            aimTargetAngle = goalAngle;
            shotDistanceIn = 0;
        }
        leadOffsetDeg = wrap180(aimTargetAngle - goalAngle);
        int targetPositionTicks = (int) (reachableAngle(aimTargetAngle, getCurrentAngle()) * TICKS_PER_DEGREE);
        turretMotor.setTargetPosition(targetPositionTicks);
        setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(AIM_MAX_POWER);
    }

    /**
     * Picks the turret angle for a direction: inside the travel limits and, if the direction can be
     * reached two ways, the one nearest the current angle. Directions in the gap behind the
     * limits go to whichever limit points closest.
     */
    static double reachableAngle(double desired, double current) {
        clampAngle(desired); // throws on a non-finite angle or bad limits
        double wrapped = wrap180(desired);
        double best = Double.NaN;
        for (int k = -1; k <= 1; k++) {
            double candidate = wrapped + 360.0 * k;
            if (candidate >= MIN_ANGLE && candidate <= MAX_ANGLE
                    && (Double.isNaN(best) || Math.abs(candidate - current) < Math.abs(best - current))) {
                best = candidate;
            }
        }
        if (!Double.isNaN(best)) return best;
        double toMax = Math.abs(wrap180(wrapped - MAX_ANGLE));
        double toMin = Math.abs(wrap180(wrapped - MIN_ANGLE));
        return toMax <= toMin ? MAX_ANGLE : MIN_ANGLE;
    }

    /**
     * Limelight camera-space point (x right, y down, z out of the lens) to a level frame
     * {right, forward, up} for a camera tilted up by pitchDeg. Same units as the input.
     */
    static double[] cameraToLevel(double[] p, double pitchDeg) {
        double t = Math.toRadians(pitchDeg);
        double forward = p[2] * Math.cos(t) + p[1] * Math.sin(t);
        double up = p[2] * Math.sin(t) - p[1] * Math.cos(t);
        return new double[] {p[0], forward, up};
    }

    /**
     * Shooting while moving: the ball keeps the robot's velocity, so aim at a virtual goal
     * shifted back by velocity x flight time. Returns {turretAngle, horizontal distance} for
     * that virtual goal. Velocity is in the robot frame (in/s, forward and left).
     */
    static double[] leadSolution(double turretAngle, double distanceIn, double vForward, double vLeft,
                                 double gain, double[][] rows) {
        double s = turretSign();
        double phi = Math.toRadians(-s * turretAngle); // counter-clockwise from straight ahead
        double gx = distanceIn * Math.cos(phi);
        double gy = distanceIn * Math.sin(phi);
        double vx = gx, vy = gy, d = distanceIn;
        for (int i = 0; i < 3; i++) { // flight time depends on distance, so iterate
            double t = ShotTable.flightTime(rows, d);
            vx = gx - gain * vForward * t;
            vy = gy - gain * vLeft * t;
            d = Math.hypot(vx, vy);
        }
        double angle = -s * Math.toDegrees(Math.atan2(vy, vx));
        return new double[] {turretAngle + wrap180(angle - turretAngle), d};
    }

    /** Robot velocity in the robot frame (in/s), from odometry. Zero = no lead. */
    public void setRobotVelocity(double forward, double left) {
        robotVForward = forward;
        robotVLeft = left;
    }

    /** Horizontal distance to the lead-compensated aim point (in), 0 if unknown. */
    public double getShotDistanceIn() {
        return shotDistanceIn;
    }

    /** Horizontal distance to the goal itself (in), 0 if unknown. */
    public double getGoalDistanceIn() {
        return goalDistanceIn;
    }

    /** Seconds since the goal was last seen with a distance; infinite if not since auto-aim started. */
    public double secondsSinceGoalSeen() {
        return lastFixNanos < 0 ? Double.POSITIVE_INFINITY : (System.nanoTime() - lastFixNanos) / 1e9;
    }

    /** How far the turret is from where auto-aim wants it, degrees. */
    public double getAimErrorDeg() {
        return Math.abs(wrap180(aimTargetAngle - getCurrentAngle()));
    }

    public boolean isAutoAimEnabled() {
        return autoAimEnabled;
    }

    /** +1 when positive turret angles turn the turret clockwise (seen from above), else -1. */
    static double turretSign() {
        return TURRET_POSITIVE_IS_CLOCKWISE ? 1.0 : -1.0;
    }

    /**
     * Field bearing of the target, in turret degrees, independent of where the robot points.
     * IMU yaw is counter-clockwise positive; Limelight tx is positive when the target is right of center.
     */
    static double fieldBearing(double turretAngle, double tx, double robotHeading) {
        double s = turretSign();
        return wrap180(turretAngle + s * tx - s * robotHeading);
    }

    /** Turret angle that points at a field bearing for the given robot heading. */
    static double turretAngleFor(double bearing, double robotHeading) {
        return wrap180(bearing + turretSign() * robotHeading);
    }

    static double wrap180(double angle) {
        angle = angle % 360.0;
        if (angle > 180) angle -= 360;
        if (angle <= -180) angle += 360;
        return angle;
    }

    private void recordHistory(double heading, double turretAngle) {
        historyIndex = (historyIndex + 1) % HISTORY_SIZE;
        historyNanos[historyIndex] = System.nanoTime();
        historyHeading[historyIndex] = heading;
        historyTurret[historyIndex] = turretAngle;
        if (historyCount < HISTORY_SIZE) historyCount++;
    }

    /** {heading, turretAngle} recorded closest to (at or before) the given time. */
    private double[] historyAt(long nanos) {
        int index = historyIndex;
        for (int i = 0; i < historyCount; i++) {
            int candidate = (historyIndex - i + HISTORY_SIZE) % HISTORY_SIZE;
            index = candidate;
            if (historyNanos[candidate] <= nanos) break;
        }
        return new double[] {historyHeading[index], historyTurret[index]};
    }

    private double getYawRateDegPerSec() {
        try {
            return imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate;
        } catch (Exception e) {
            return 0.0;
        }
    }

    /** Where the goal is in this camera frame: the centre of our alliance's highest visible tag cluster. */
    static final class ClusterFix {
        final int clusterIndex; // 0: IDs 30-33, 1: 34-37, 2: 38-41, 3: 42-45
        final int tagCount;
        final double txDeg; // horizontal angle to the cluster centre, positive = right
        final double distanceM; // camera to cluster centre, 0 if 3D data is unavailable
        final double horizontalIn; // level distance from the camera to the cluster centre, 0 if unknown
        final double heightAboveCameraIn;

        ClusterFix(int clusterIndex, int tagCount, double txDeg, double distanceM,
                   double horizontalIn, double heightAboveCameraIn) {
            this.clusterIndex = clusterIndex;
            this.tagCount = tagCount;
            this.txDeg = txDeg;
            this.distanceM = distanceM;
            this.horizontalIn = horizontalIn;
            this.heightAboveCameraIn = heightAboveCameraIn;
        }
    }

    static boolean allianceOwnsCluster(int clusterIndex) {
        switch (ALLIANCE) {
            case RED: return clusterIndex == 0 || clusterIndex == 1;
            case BLUE: return clusterIndex == 2 || clusterIndex == 3;
            default: return clusterIndex >= 0 && clusterIndex <= 3;
        }
    }

    private ClusterFix findCluster(LLResult result) {
        if (result == null || !result.isValid()) return null;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;

        // Group visible tags by cluster, keep our alliance's, pick the highest (largest ty).
        int bestCluster = -1;
        double bestTy = Double.NEGATIVE_INFINITY;
        for (int cluster = 0; cluster < 4; cluster++) {
            if (!allianceOwnsCluster(cluster)) continue;
            double tySum = 0;
            int count = 0;
            for (LLResultTypes.FiducialResult f : fiducials) {
                if (clusterOf(f.getFiducialId()) == cluster) {
                    tySum += f.getTargetYDegrees();
                    count++;
                }
            }
            if (count > 0 && tySum / count > bestTy) {
                bestTy = tySum / count;
                bestCluster = cluster;
            }
        }
        if (bestCluster < 0) return null;

        List<Double> offsets = new java.util.ArrayList<>();
        List<double[]> positions = new java.util.ArrayList<>();
        double txSum = 0;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (clusterOf(f.getFiducialId()) != bestCluster) continue;
            txSum += f.getTargetXDegrees();
            Pose3D pose = f.getTargetPoseCameraSpace();
            if (pose == null) continue;
            double x = pose.getPosition().x, y = pose.getPosition().y, z = pose.getPosition().z;
            if (z <= 0) continue; // no 3D data
            offsets.add(TAG_OFFSETS_M[(f.getFiducialId() - FIRST_TAG_ID) % 4]);
            positions.add(new double[] {x, y, z});
        }
        int tagCount = Math.max(offsets.size(), 1);
        if (positions.isEmpty()) {
            // No 3D poses: average the tags' angles (exact only when the tags straddle the centre).
            int seen = 0;
            for (LLResultTypes.FiducialResult f : fiducials) if (clusterOf(f.getFiducialId()) == bestCluster) seen++;
            return new ClusterFix(bestCluster, seen, txSum / seen, 0, 0, 0);
        }
        double[] offsetArray = new double[offsets.size()];
        for (int i = 0; i < offsetArray.length; i++) offsetArray[i] = offsets.get(i);
        double[] center = clusterCenter(offsetArray, positions.toArray(new double[0][]));
        double[] level = cameraToLevel(center, CAMERA_PITCH_DEG);
        double txDeg = Math.toDegrees(Math.atan2(level[0], level[1]));
        double distance = Math.sqrt(center[0] * center[0] + center[1] * center[1] + center[2] * center[2]);
        return new ClusterFix(bestCluster, tagCount, txDeg, distance,
            Math.hypot(level[0], level[1]) / 0.0254, level[2] / 0.0254);
    }

    static int clusterOf(int tagId) {
        if (tagId < FIRST_TAG_ID || tagId >= FIRST_TAG_ID + 16) return -1;
        return (tagId - FIRST_TAG_ID) / 4;
    }

    /**
     * Cluster centre in camera space from tags at known offsets along the sticker.
     * With 2+ tags the sticker line is fitted through them, so the result does not depend on
     * which tags are visible. With one tag, that tag's position is used (up to 6.5 in off).
     */
    static double[] clusterCenter(double[] offsets, double[][] positions) {
        int n = offsets.length;
        double meanOffset = 0;
        double[] meanPos = new double[3];
        for (int i = 0; i < n; i++) {
            meanOffset += offsets[i] / n;
            for (int k = 0; k < 3; k++) meanPos[k] += positions[i][k] / n;
        }
        double spread = 0;
        double[] slope = new double[3];
        for (int i = 0; i < n; i++) {
            double d = offsets[i] - meanOffset;
            spread += d * d;
            for (int k = 0; k < 3; k++) slope[k] += d * (positions[i][k] - meanPos[k]);
        }
        if (spread < 1e-9) return meanPos;
        double[] center = new double[3];
        for (int k = 0; k < 3; k++) center[k] = meanPos[k] - meanOffset * slope[k] / spread;
        return center;
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
        if (!autoAimEnabled && enable) {
            targetWasVisible = false; // start from a fresh camera fix, not an old bearing
            lastFrameNanos = -1;
            lastFixNanos = -1;
            goalDistanceIn = 0;
            shotDistanceIn = 0;
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
        telemetry.addData("Robot Heading / Turn Rate", "%.1f° / %.0f°/s",
            getRobotHeading(), getYawRateDegPerSec());
        if (autoAimEnabled) {
            telemetry.addData("Aim Bearing / Turret Target", "%.1f° / %.1f°",
                targetBearingField, aimTargetAngle);
            telemetry.addData("Alliance / Aim Source", "%s / %s", ALLIANCE, AIM_SOURCE);
        }
        if (autoAimEnabled && limelight != null) {
            LLResult latest = limelight.getLatestResult();
            StringBuilder ids = new StringBuilder();
            if (latest != null && latest.getFiducialResults() != null) {
                for (LLResultTypes.FiducialResult f : latest.getFiducialResults()) {
                    if (ids.length() > 0) ids.append(',');
                    ids.append(f.getFiducialId());
                }
            }
            telemetry.addData("Limelight link", "%s | connected %b | running %b | last data %d ms ago",
                limelight.getConnectionInfo(), limelight.isConnected(), limelight.isRunning(),
                limelight.getTimeSinceLastUpdate());
            telemetry.addData("Limelight status", limelight.getStatus() == null ? "none"
                : String.format("pipeline %d (%s) | %.0f fps",
                    limelight.getStatus().getPipelineIndex(), limelight.getStatus().getPipelineType(),
                    limelight.getStatus().getFps()));
            telemetry.addData("Limelight pipeline / valid / tags", "%d / %b / %s",
                latest == null ? -1 : latest.getPipelineIndex(),
                latest != null && latest.isValid(),
                ids.length() == 0 ? "none" : ids.toString());
            if (lastClusterFix != null) {
                int firstId = FIRST_TAG_ID + lastClusterFix.clusterIndex * 4;
                telemetry.addData("Goal Tags / Seen", "%d-%d / %d", firstId, firstId + 3, lastClusterFix.tagCount);
                telemetry.addData("Goal Distance", lastClusterFix.distanceM > 0
                    ? String.format("%.2f m (horizontal %.0f in, goal height %.0f in)", lastClusterFix.distanceM,
                        goalDistanceIn, lastClusterFix.heightAboveCameraIn + CAMERA_HEIGHT_IN)
                    : "no 3D data (enable full 3D)");
                telemetry.addData("Lead / Shot Distance", "%.1f° / %.0f in  (v fwd %.0f, left %.0f in/s)",
                    leadOffsetDeg, shotDistanceIn, robotVForward, robotVLeft);
            }
        }
        if (autoAimEnabled && AIM_SOURCE == AimSource.LIMELIGHT) {
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
