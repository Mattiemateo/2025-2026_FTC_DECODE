package org.firstinspires.ftc.teamcode;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.TurretController.Alliance;

/**
 * The three BIOBUZZ autonomous plans. Field spots live in AutoField (written for RED, mirrored for
 * BLUE). Driving uses odometry only (point to point), shooting uses the TeleOp's auto-aim and
 * READY checks. AUTO is 30 s; scoring stops at PARK_BY_SEC so there is always time to park.
 *
 *   HIVE_FRONT: start at the upward CELL (RED: C1). Auto-aim finds it, leave the wall, shoot the 4 preloads,
 *     park in the LOADING ZONE.
 *   SECOND_CELL_TIP: start at the downward CELL (RED: C6) and watch it. When our partner tips the HIVE (that
 *     CELL comes up), shoot the preloads into it, collect the 4 GARDEN POLLEN, shoot them from 1.2 m
 *     and park. Not tipped within TIP_CUTOFF_SEC: shoot the preloads into the original upward CELL
 *     from 1.2 m (drive under the HIVE, then right, shoot diagonally), then park.
 *   SECOND_CELL_PARK: the same, but not tipped in time = just park.
 */
public abstract class BiobuzzAuto extends LinearOpMode {
    enum Plan { HIVE_FRONT, SECOND_CELL_TIP, SECOND_CELL_PARK }

    // Timing (seconds since Start)
    public static double TIP_CUTOFF_SEC = 9.0;
    public static double PARK_BY_SEC = 24.0;
    public static double AUTO_END_SEC = 29.5;
    // Shooting: no ball sensor, so feed for FEED_SEC of READY time (enough for 4 balls).
    public static double FEED_SEC = 3.0;
    public static double SHOOT_TIMEOUT_SEC = 6.0;
    public static boolean INTAKE_WHILE_SHOOTING = true; // helps push the last balls into the transfer
    // Driving
    public static double DRIVE_POWER = 0.5;
    public static double PICKUP_POWER = 0.2;
    public static double VIA_TOLERANCE_IN = 4.0;
    public static double STEP_TIMEOUT_SEC = 5.0;
    public static double SETTLE_SEC = 0.15;
    // Tip watch: the watched cluster this high above the camera means "that CELL is up". A down
    // CELL's tags are about 21 in above the camera, an upward CELL's about 43 in.
    public static double TIP_HEIGHT_IN = 32.0;
    public static double TIP_CONFIRM_SEC = 0.25;
    // No odometry: leave by driving forward for this long.
    public static double BLIND_LEAVE_POWER = 0.3, BLIND_LEAVE_SEC = 0.5;

    // Handed to the TeleOp, so it keeps the odometry pose instead of assuming the start spot.
    static double[] handoffPose = null;
    static long handoffNanos = 0;
    public static double HANDOFF_MAX_AGE_SEC = 120.0;

    protected final BiobuzzRobot robot = new BiobuzzRobot();
    private final ElapsedTime clock = new ElapsedTime();
    private Alliance alliance;
    private String step = "";
    private boolean autoAim = false;
    private double spinRpm = 0;
    private double intakePower = 0;
    private Pose pose;

    abstract Plan plan();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap, telemetry);
        while (opModeInInit()) {
            BiobuzzRobot.selectAlliance(gamepad1);
            telemetry.addData("Plan", plan());
            telemetry.addData("Alliance", "%s  (Circle = RED, Cross = BLUE)", TurretController.ALLIANCE);
            robot.turretInitCheck(gamepad1, telemetry);
            telemetry.addLine(startHint(plan(), TurretController.ALLIANCE));
            telemetry.addLine("Preload 4 POLLEN.");
            if (robot.localizer == null) telemetry.addData("Odometry OFF", robot.odometryError);
            telemetry.update();
            idle();
        }
        alliance = TurretController.ALLIANCE;
        if (!opModeIsActive()) return;
        clock.reset();
        try {
            if (alliance == Alliance.ANY) {
                telemetry.addLine("No alliance picked in INIT: doing nothing.");
                telemetry.update();
                while (opModeIsActive()) idle();
                return;
            }
            double[] start = plan() == Plan.HIVE_FRONT ? AutoField.startAtUpCell() : AutoField.startAtDownCell();
            if (robot.localizer == null) {
                blindAuto();
            } else {
                double[] s = AutoField.forAlliance(start, alliance);
                robot.localizer.setPose(new Pose(s[0], s[1], Math.toRadians(s[2])));
                if (plan() == Plan.HIVE_FRONT) hiveFront(start);
                else secondCell();
            }
            park();
        } finally {
            if (pose != null) {
                handoffPose = new double[] {pose.x(), pose.y(), Math.toDegrees(pose.heading())};
                handoffNanos = System.nanoTime();
            }
            robot.stopAll();
        }
    }

    private void hiveFront(double[] start) {
        setAutoAim(true); // sees the upward CELL from the start spot and remembers it
        driveTo("leave the wall", AutoField.leaveSpot(start), false, DRIVE_POWER, PARK_BY_SEC);
        shoot("shoot preloads");
    }

    private void secondCell() {
        boolean tipped = watchForTip();
        if (tipped) {
            setAutoAim(true);
            shoot("shoot preloads (partner tipped)");
            collectGarden();
            driveRoute("to the 1.2 m shot", AutoField.gardenShotRoute(), PARK_BY_SEC);
            shoot("shoot GARDEN POLLEN");
        } else if (plan() == Plan.SECOND_CELL_TIP) {
            setAutoAim(true);
            driveRoute("fallback: under the HIVE to the 1.2 m shot", AutoField.fallbackRoute(), PARK_BY_SEC);
            shoot("fallback: shoot preloads");
        }
    }

    /** Turret on the downward CELL; true once it comes up, false at TIP_CUTOFF_SEC. */
    private boolean watchForTip() {
        setAutoAim(false);
        int cluster = AutoField.startDownCluster(alliance);
        double[] cell = mine(AutoField.startDownCell());
        spinRpm = ShotTable.rpm(50); // spin up now, so the shot can go as soon as it tips
        TipWatch watch = new TipWatch();
        while (opModeIsActive() && clock.seconds() < TIP_CUTOFF_SEC) {
            step = String.format("watching for the tip (%.0f s left)", TIP_CUTOFF_SEC - clock.seconds());
            if (pose != null) {
                robot.turret.setManualTargetAngle(TurretController.turretAngleToPoint(
                    pose.x(), pose.y(), Math.toDegrees(pose.heading()), cell[0], cell[1]));
            }
            robot.drive(0, 0, 0, 0);
            double height = robot.turret.clusterHeightAboveCameraIn(cluster);
            telemetry.addData("Watched CELL height", Double.isNaN(height) ? "not seen"
                : String.format("%.0f in above camera (tip at %.0f)", height, TIP_HEIGHT_IN));
            tick();
            if (watch.update(height, clock.seconds())) return true;
        }
        return false;
    }

    private void collectGarden() {
        driveRoute("to the GARDEN", AutoField.gardenRoute(), PARK_BY_SEC);
        intakePower = BiobuzzRobot.INTAKE_POWER;
        driveTo("collect GARDEN POLLEN", AutoField.gardenEnd(), false, PICKUP_POWER, PARK_BY_SEC);
        intakePower = 0;
    }

    private void park() {
        setAutoAim(false);
        spinRpm = 0;
        robot.turret.setManualTargetAngle(0); // TeleOp starts with the turret centred
        if (pose != null) {
            double[] red = AutoField.toRed(new double[] {pose.x(), pose.y(), Math.toDegrees(pose.heading())}, alliance);
            driveRoute("park in the LOADING ZONE", AutoField.parkRoute(red[0], red[1], red[2]), AUTO_END_SEC);
        }
        while (opModeIsActive()) {
            step = "done";
            robot.drive(0, 0, 0, 0);
            tick();
        }
    }

    /** No odometry: leave the wall by time; HIVE_FRONT still shoots with the camera. */
    private void blindAuto() {
        step = "no odometry: leave by time";
        ElapsedTime t = new ElapsedTime();
        while (opModeIsActive() && t.seconds() < BLIND_LEAVE_SEC) {
            robot.drive(BLIND_LEAVE_POWER, 0, 0, 1.0);
            tick();
        }
        robot.drive(0, 0, 0, 0);
        if (plan() == Plan.HIVE_FRONT) {
            setAutoAim(true);
            shoot("shoot preloads (camera only)");
        }
    }

    /** Feed while READY until FEED_SEC of feeding, SHOOT_TIMEOUT_SEC, or PARK_BY_SEC. */
    private void shoot(String name) {
        ElapsedTime t = new ElapsedTime();
        double fed = 0, last = 0;
        while (opModeIsActive() && clock.seconds() < PARK_BY_SEC && t.seconds() < SHOOT_TIMEOUT_SEC && fed < FEED_SEC) {
            robot.drive(0, 0, 0, 0);
            tick();
            double distance = robot.turret.getShotDistanceIn();
            String blocker = BIOBUZZ.shotBlocker(robot.turret.isTargetInBlindSpot(), robot.turret.getAimErrorDeg(),
                robot.flywheel.getCurrentRPM(), spinRpm, robot.turret.goalAgeForShot(), distance);
            boolean fire = blocker == null;
            robot.transfer.setPower(fire ? BiobuzzRobot.TRANSFER_POWER : 0);
            robot.intake.setPower(fire && INTAKE_WHILE_SHOOTING ? BiobuzzRobot.INTAKE_POWER : 0);
            double now = t.seconds();
            if (fire) fed += now - last;
            last = now;
            step = String.format("%s: %s (fed %.1f/%.1f s)", name, fire ? "FIRING" : "wait: " + blocker, fed, FEED_SEC);
        }
        robot.transfer.setPower(0);
        robot.intake.setPower(0);
    }

    private void driveRoute(String name, double[][] redPoints, double abortAt) {
        for (int i = 0; i < redPoints.length; i++) {
            boolean last = i == redPoints.length - 1;
            // A leg that times out (blocked?) moves on to the next point; out of time stops the route.
            if (!driveTo(name, redPoints[i], !last, DRIVE_POWER, abortAt)
                && (!opModeIsActive() || clock.seconds() >= abortAt)) return;
        }
    }

    /**
     * Drive to a RED-frame spot. Pass-through (via) points: full speed, done within
     * VIA_TOLERANCE_IN. Final points: slow down and settle on the spot and heading.
     * False when stopped, timed out or out of time.
     */
    private boolean driveTo(String name, double[] red, boolean via, double maxPower, double abortAt) {
        double[] target = mine(red);
        ElapsedTime stepTimer = new ElapsedTime();
        ElapsedTime settle = new ElapsedTime();
        step = name;
        while (opModeIsActive() && clock.seconds() < abortAt) {
            tick();
            if (pose == null) break;
            double h = Math.toDegrees(pose.heading());
            double[] cmd = OdometryTileTest.driveCommand(pose.x(), pose.y(), h, target[0], target[1], target[2], maxPower);
            double distance = Math.hypot(target[0] - pose.x(), target[1] - pose.y());
            if (via) {
                double translation = Math.hypot(cmd[0], cmd[1]);
                if (translation > 0) {
                    cmd[0] *= maxPower / translation;
                    cmd[1] *= maxPower / translation;
                }
            }
            robot.drive(cmd[0], cmd[1], cmd[2], 1.0);
            if (via) {
                if (distance < VIA_TOLERANCE_IN) return true;
            } else {
                boolean arrived = distance < OdometryTileTest.POSITION_TOLERANCE_IN
                    && Math.abs(TurretController.wrap180(target[2] - h)) < OdometryTileTest.HEADING_TOLERANCE_DEG;
                if (!arrived) settle.reset();
                if (settle.seconds() > SETTLE_SEC) {
                    robot.drive(0, 0, 0, 0);
                    return true;
                }
            }
            if (stepTimer.seconds() > STEP_TIMEOUT_SEC) {
                step = name + " (TIMED OUT)";
                break;
            }
        }
        robot.drive(0, 0, 0, 0);
        return false;
    }

    private void setAutoAim(boolean on) {
        autoAim = on;
        robot.turret.enableAutoAim(on);
    }

    /** One loop of everything that isn't driving: odometry, turret, flywheel, intake, telemetry. */
    private void tick() {
        pose = robot.updateOdometry();
        robot.turret.update();
        if (autoAim) {
            double distance = robot.turret.getShotDistanceIn();
            if (distance > 0) spinRpm = ShotTable.rpm(distance);
        }
        robot.flywheel.setTargetRPM(spinRpm);
        robot.intake.setPower(intakePower); // shoot() overrides this right after
        telemetry.addData("Plan / Alliance", "%s / %s", plan(), alliance);
        telemetry.addData("Time", "%.1f s", clock.seconds());
        telemetry.addData("Step", step);
        if (pose != null) {
            telemetry.addData("Pose (in, deg)", "x %.1f  y %.1f  h %.0f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
        }
        telemetry.addData("Flywheel (target/measured)", "%.0f / %.0f", spinRpm, robot.flywheel.getCurrentRPM());
        telemetry.update();
    }

    /** A RED-frame spot ({x, y} or {x, y, heading}) for our alliance. */
    private double[] mine(double[] red) {
        return AutoField.forAlliance(red, alliance);
    }

    /** The pose the last autonomous ended at, if it ended less than HANDOFF_MAX_AGE_SEC ago. */
    static boolean hasHandoff() {
        return handoffPose != null && (System.nanoTime() - handoffNanos) / 1e9 <= HANDOFF_MAX_AGE_SEC;
    }

    /** Take the handoff pose {x, y, headingDeg} (once), or null. */
    static double[] takeHandoff() {
        double[] p = hasHandoff() ? handoffPose : null;
        handoffPose = null;
        return p;
    }

    static String startHint(Plan plan, Alliance alliance) {
        boolean atUp = plan == Plan.HIVE_FRONT;
        boolean blue = alliance == Alliance.BLUE;
        // RED: Auto 1 in C1 (audience wall) below the upward CELL C3; Auto 2/3 in C6 (far wall) above C4.
        // BLUE is the same turned around: D6 above D4, and D1 below D3.
        boolean audienceWall = atUp != blue;
        return String.format("Start: tile %s, back against the %s wall, facing the HIVE, lined up with the %s CELL %s.",
            blue ? (atUp ? "D6" : "D1") : (atUp ? "C1" : "C6"), audienceWall ? "audience" : "far",
            atUp ? "UPWARD" : "DOWNWARD", blue ? (atUp ? "D4" : "D3") : (atUp ? "C3" : "C4"));
    }

    /**
     * Tip detection from the watched cluster's height above the camera. Tipped once it has been
     * above TIP_HEIGHT_IN for TIP_CONFIRM_SEC (first and latest high reading that far apart).
     * Frames where it isn't seen (NaN) don't reset it; a low reading does.
     */
    static final class TipWatch {
        private double highSince = Double.NaN;

        boolean update(double heightIn, double now) {
            if (Double.isNaN(heightIn)) return false; // not seen this frame: wait for the next one
            if (heightIn < TIP_HEIGHT_IN) {
                highSince = Double.NaN;
                return false;
            }
            if (Double.isNaN(highSince)) highSince = now;
            return now - highSince >= TIP_CONFIRM_SEC;
        }
    }
}
