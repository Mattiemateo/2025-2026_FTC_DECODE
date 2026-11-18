package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.TurretController.Alliance;

/**
 * BIOBUZZ field geometry for odometry auto-aim (Competition Manual TU08, section 9).
 *
 * Field frame: origin at the field centre, inches.
 *   +X points at the BLUE alliance wall (right, as the audience sees it); red is on the -X wall.
 *   +Y points away from the audience.
 *   Heading 0 = facing +X, counter-clockwise positive (same as Pedro and the IMU).
 */
public final class FieldAim {
    /** Which CELL of our HIVE is tipped up (the one to shoot into). Swaps every time the HIVE tips. */
    public enum Cell { FAR, AUDIENCE }

    // HIVE centre to centre is 25.5 in. (Figure 9-10): red HIVE at -12.75, blue at +12.75.
    public static double HIVE_X = 12.75;
    // Upward CELL centre, measured along the field from the pivot. Each CELL sits ~15.4 in. out
    // on its arm (Figure 9-9) and the arm is tipped 30 deg (Figure 9-10): 15.4 * cos(30) = 13.4.
    public static double CELL_Y = 13.4;

    // Start pose: back of the robot against our own alliance wall, robot centre on the
    // field's centre line (Y = START_Y), facing the HIVE. Turret centred (0) before INIT.
    public static double ROBOT_HALF_LENGTH = 9.0; // robot centre to back bumper (18 in. robot)
    public static double START_Y = 0.0;
    // Turret pivot ahead of the robot centre, inches (negative = behind the centre).
    public static double TURRET_FORWARD_OFFSET = 0.0;

    static final double HALF_FIELD = 72.0;

    private FieldAim() {
    }

    /** {x, y} of the upward CELL of our HIVE. */
    public static double[] goal(Alliance alliance, Cell cell) {
        double x = alliance == Alliance.RED ? -HIVE_X : HIVE_X;
        double y = cell == Cell.FAR ? CELL_Y : -CELL_Y;
        return new double[] {x, y};
    }

    /** {x, y, headingDeg} where the robot starts, for RED or BLUE. */
    public static double[] startPose(Alliance alliance) {
        double x = HALF_FIELD - ROBOT_HALF_LENGTH;
        return alliance == Alliance.RED
            ? new double[] {-x, START_Y, 0.0}
            : new double[] {x, START_Y, 180.0};
    }

    /** {x, y} of the turret pivot for a robot pose. */
    public static double[] turretPosition(double robotX, double robotY, double headingDeg) {
        double h = Math.toRadians(headingDeg);
        return new double[] {
            robotX + TURRET_FORWARD_OFFSET * Math.cos(h),
            robotY + TURRET_FORWARD_OFFSET * Math.sin(h)
        };
    }
}
