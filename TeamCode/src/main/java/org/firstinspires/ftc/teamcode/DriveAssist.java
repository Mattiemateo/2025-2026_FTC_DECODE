package org.firstinspires.ftc.teamcode;

/**
 * Driver assists for the mecanum drive. Stick in, drive command out.
 *
 *   A. Stick shaping: dead zone, a "straight zone" of +/- SNAP_DEG around pure forward/back and
 *      pure left/right that removes the small sideways (or forward) part, and a response curve.
 *   B. Heading hold (gyro): with the turn stick released, lock the heading once the robot has
 *      stopped turning and correct unwanted rotation.
 *   C. Line hold (odometry): while in a straight zone, keep the robot on the line it started on.
 *   D. Field-relative (gyro, optional): stick forward = away from the drivers.
 *
 * Frames: odometry field frame (x, y, heading counter-clockwise). Robot frame: forward, left.
 * Output for BIOBUZZ.drivePowers: {forward, strafeRight, turnClockwise}.
 */
public class DriveAssist {
    // A. Stick shaping
    public static double DEADZONE = 0.05;
    public static double SNAP_DEG = 8.0;
    // 0 = linear, 1 = fully cubic. In between: finer control near the centre, full speed at full stick.
    public static double CURVE = 0.5;
    // B. Heading hold
    public static double HEADING_KP = 0.02; // turn power per degree of error
    public static double HEADING_KD = 0.002; // turn power per deg/s of turn rate (damping)
    public static double HEADING_CAPTURE_RATE_DEG_PER_SEC = 20.0; // lock once turning slower than this
    public static double HEADING_MAX_POWER = 0.4;
    // C. Line hold
    public static double LINE_KP = 0.03; // power per inch off the line
    public static double LINE_KV = 0.005; // power per in/s of sideways speed
    public static double LINE_MAX_POWER = 0.3;
    public static double LINE_MIN_STICK = 0.15; // only hold a line while actually driving

    public enum Snap { NONE, FORWARD_BACK, LEFT_RIGHT }

    // Toggles (set from BIOBUZZ)
    public boolean assistsOn = true; // A, B, C
    public boolean fieldRelative = false; // D
    private double fieldReferenceDeg = 0.0; // IMU heading that counts as "away from the drivers"

    // State
    private double headingTarget = Double.NaN;
    private boolean lineActive = false;
    private Snap lineSnap = Snap.NONE;
    private double lineSign = 0;
    private double lineOriginX, lineOriginY, lineDirX, lineDirY;

    // Telemetry
    private Snap lastSnap = Snap.NONE;
    private double lastLineErrorIn = 0;

    /** "Away from the drivers" = this IMU heading (call at Start, and on R3). */
    public void setFieldReference(double imuHeadingDeg) {
        fieldReferenceDeg = imuHeadingDeg;
        headingTarget = Double.NaN;
        lineActive = false;
    }

    /**
     * One loop of driving.
     *
     * @param stickForward  -left_stick_y
     * @param stickRight    left_stick_x
     * @param stickTurn     right_stick_x (positive = clockwise)
     * @param imuHeadingDeg gyro heading, counter-clockwise
     * @param imuRateDegPerSec gyro turn rate, counter-clockwise
     * @param odometryOn    pose and velocity below are valid and trusted
     * @param x             odometry x (in)
     * @param y             odometry y (in)
     * @param odoHeadingDeg odometry heading (deg, counter-clockwise)
     * @param vForward      robot-frame forward speed (in/s)
     * @param vLeft         robot-frame left speed (in/s)
     */
    public double[] update(double stickForward, double stickRight, double stickTurn,
                           double imuHeadingDeg, double imuRateDegPerSec,
                           boolean odometryOn, double x, double y, double odoHeadingDeg,
                           double vForward, double vLeft) {
        // Translation in the command frame (robot, or drivers' view when field-relative).
        double f = stickForward, r = stickRight, turn = stickTurn;
        Snap snap = Snap.NONE;
        if (assistsOn) {
            double[] shaped = shapeTranslation(f, r);
            f = shaped[0];
            r = shaped[1];
            snap = snapOf(shaped[2]);
            turn = shapeAxis(turn);
        }
        lastSnap = snap;

        // D. Field-relative: rotate the drivers' view into the robot frame.
        if (fieldRelative) {
            double[] robot = driverToRobot(f, r, imuHeadingDeg - fieldReferenceDeg);
            f = robot[0];
            r = robot[1];
        }

        // B. Heading hold.
        if (assistsOn && turn == 0) {
            if (Double.isNaN(headingTarget) && Math.abs(imuRateDegPerSec) < HEADING_CAPTURE_RATE_DEG_PER_SEC) {
                headingTarget = imuHeadingDeg;
            }
            if (!Double.isNaN(headingTarget)) {
                turn = headingHoldTurn(headingTarget, imuHeadingDeg, imuRateDegPerSec);
            }
        } else {
            headingTarget = Double.NaN;
        }

        // C. Line hold, only while driving in a straight zone without turning.
        boolean driving = Math.hypot(f, r) >= LINE_MIN_STICK;
        if (assistsOn && odometryOn && snap != Snap.NONE && driving && stickTurnIsZero(stickTurn)) {
            double h = Math.toRadians(odoHeadingDeg);
            // Commanded direction in the field frame (robot frame: forward f, left -r).
            double norm = Math.hypot(f, r);
            double dirX = (f * Math.cos(h) + r * Math.sin(h)) / norm;
            double dirY = (f * Math.sin(h) - r * Math.cos(h)) / norm;
            double sign = snap == Snap.FORWARD_BACK ? Math.signum(stickForward) : Math.signum(stickRight);
            if (!lineActive || lineSnap != snap || lineSign != sign) {
                lineActive = true;
                lineSnap = snap;
                lineSign = sign;
                lineOriginX = x;
                lineOriginY = y;
                lineDirX = dirX;
                lineDirY = dirY;
            }
            double[] fieldVel = robotToField(vForward, vLeft, h);
            double[] corr = lineCorrection(x - lineOriginX, y - lineOriginY, fieldVel[0], fieldVel[1],
                lineDirX, lineDirY);
            lastLineErrorIn = corr[2];
            // Field-frame correction into the robot frame.
            double corrForward = corr[0] * Math.cos(h) + corr[1] * Math.sin(h);
            double corrLeft = -corr[0] * Math.sin(h) + corr[1] * Math.cos(h);
            f += corrForward;
            r -= corrLeft;
        } else {
            lineActive = false;
            lastLineErrorIn = 0;
        }
        return new double[] {f, r, turn};
    }

    private static boolean stickTurnIsZero(double stickTurn) {
        return Math.abs(stickTurn) < DEADZONE;
    }

    /** {forward, right, snapCode} after dead zone, straight-zone snap and response curve. */
    static double[] shapeTranslation(double forward, double right) {
        double magnitude = Math.hypot(forward, right);
        if (magnitude < DEADZONE) return new double[] {0, 0, 0};
        double angle = Math.toDegrees(Math.atan2(right, forward)); // 0 = forward, 90 = right
        double fromForwardAxis = Math.min(Math.abs(angle), 180 - Math.abs(angle));
        double fromSideAxis = Math.abs(90 - Math.abs(angle));
        int snapCode = 0;
        if (fromForwardAxis <= SNAP_DEG) {
            right = 0;
            snapCode = 1;
        } else if (fromSideAxis <= SNAP_DEG) {
            forward = 0;
            snapCode = 2;
        }
        double shapedMagnitude = curve(Math.min(1.0, (magnitude - DEADZONE) / (1 - DEADZONE)));
        double remaining = Math.hypot(forward, right);
        return new double[] {forward / remaining * shapedMagnitude, right / remaining * shapedMagnitude, snapCode};
    }

    static Snap snapOf(double code) {
        return code == 1 ? Snap.FORWARD_BACK : code == 2 ? Snap.LEFT_RIGHT : Snap.NONE;
    }

    static double shapeAxis(double value) {
        double magnitude = Math.abs(value);
        if (magnitude < DEADZONE) return 0;
        return Math.copySign(curve(Math.min(1.0, (magnitude - DEADZONE) / (1 - DEADZONE))), value);
    }

    static double curve(double x) {
        return CURVE * x * x * x + (1 - CURVE) * x;
    }

    /** Drivers' view {forward, right} to robot frame {forward, right}; offset = robot heading - reference. */
    static double[] driverToRobot(double forward, double right, double offsetDeg) {
        double a = Math.toRadians(offsetDeg);
        double left = -right;
        double robotForward = forward * Math.cos(a) + left * Math.sin(a);
        double robotLeft = -forward * Math.sin(a) + left * Math.cos(a);
        return new double[] {robotForward, -robotLeft};
    }

    /** Clockwise turn power that holds the heading target. */
    static double headingHoldTurn(double targetDeg, double headingDeg, double rateDegPerSec) {
        double error = TurretController.wrap180(targetDeg - headingDeg);
        double ccw = HEADING_KP * error - HEADING_KD * rateDegPerSec;
        ccw = Math.max(-HEADING_MAX_POWER, Math.min(HEADING_MAX_POWER, ccw));
        return -ccw;
    }

    static double[] robotToField(double forward, double left, double headingRad) {
        return new double[] {
            forward * Math.cos(headingRad) - left * Math.sin(headingRad),
            forward * Math.sin(headingRad) + left * Math.cos(headingRad)
        };
    }

    /**
     * Field-frame correction {x, y, crossTrackErrorIn} that pushes back toward the line through
     * the origin along (dirX, dirY): offset (dx, dy) from the origin, field velocity (vx, vy).
     */
    static double[] lineCorrection(double dx, double dy, double vx, double vy, double dirX, double dirY) {
        double nx = -dirY, ny = dirX; // left of the line direction
        double error = dx * nx + dy * ny;
        double crossSpeed = vx * nx + vy * ny;
        double power = -(LINE_KP * error + LINE_KV * crossSpeed);
        power = Math.max(-LINE_MAX_POWER, Math.min(LINE_MAX_POWER, power));
        return new double[] {power * nx, power * ny, error};
    }

    public String status() {
        String heading = Double.isNaN(headingTarget) ? "free" : String.format("locked %.0f°", headingTarget);
        String line = lineActive ? String.format("on (%.1f in off)", lastLineErrorIn) : "off";
        return String.format("%s | %s | snap %s | heading %s | line %s",
            assistsOn ? "assists ON" : "assists OFF", fieldRelative ? "FIELD" : "ROBOT", lastSnap, heading, line);
    }
}
