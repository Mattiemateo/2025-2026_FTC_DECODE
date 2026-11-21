package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.TurretController.Alliance;

/**
 * Field spots for the autonomous OpModes (BIOBUZZ Competition Manual TU10, sections 9 and 10).
 *
 * Every spot is written for RED, as {x, y, headingDeg} in the FieldAim frame (origin at the field
 * centre, +X toward the blue wall, +Y away from the audience, heading counter-clockwise, 0 = facing
 * the blue wall). BLUE's field is RED's turned 180 degrees about the centre, so forAlliance()
 * mirrors every spot. Change the numbers here after measuring on the field.
 *
 * RED's field (Fig 9-3; start state checked on the team's field 2026-11-21):
 *   - Seen from behind our alliance wall: LOADING ZONE on the left, GARDEN in the right corner.
 *   - At the start our LEFT CELL is up (FAR, next to the LOADING ZONE) and the right one
 *     (AUDIENCE, near the GARDEN) is down.
 *   - LOADING ZONE: x -72..-61, y +24..+48 (against our alliance wall, far end).
 *   - GARDEN: 4 POLLEN in a row along the audience wall from our corner, x -71..-62, y about -70.6.
 */
public final class AutoField {
    public static double ROBOT_HALF_WIDTH = 9.0; // robot centre to its side (18 in. robot)
    // Robot centre this far from the alliance wall after "leave" (start is ROBOT_HALF_LENGTH).
    public static double LEAVE_IN = 4.0;
    // Lane the robot drives along, between partner's start spot and the HIVE legs (x about -25).
    public static double LANE_X = -40.0;
    // Park: robot centre; its back half then overlaps the LOADING ZONE (x up to -61).
    public static double PARK_X = -58.0, PARK_Y = 36.0;
    // GARDEN pickup: drive along the audience wall, intake first, from GARDEN_START_X to the corner.
    public static double GARDEN_START_X = -40.0;
    public static double GARDEN_WALL_GAP_IN = 1.0; // gap between the robot's side and the wall
    // 1.2 m shots. Bearing = direction from the CELL to the robot, degrees counter-clockwise.
    public static double SHOT_DISTANCE_IN = 47.2;
    public static double GARDEN_SHOT_BEARING_DEG = 200.0; // from the AUDIENCE CELL toward our GARDEN corner
    // Fallback: drive straight under the HIVE (the robot fits), then right, and shoot diagonally up
    // into the FAR CELL from the audience side. 260 deg keeps the robot on our half (x < 0, G402).
    public static double FALLBACK_SHOT_BEARING_DEG = 260.0;

    // Tag clusters (TurretController.ClusterFix): 0 red far, 1 red audience, 2 blue audience, 3 blue far.
    static final double HALF_FIELD = 72.0;

    private AutoField() {
    }

    /** RED spot {x, y, headingDeg} (or point {x, y}) as it is for this alliance. */
    static double[] forAlliance(double[] red, Alliance alliance) {
        if (alliance != Alliance.BLUE) return red.clone();
        if (red.length == 2) return new double[] {-red[0], -red[1]};
        return new double[] {-red[0], -red[1], TurretController.wrap180(red[2] + 180)};
    }

    /** The inverse of forAlliance (the rotation is its own inverse). */
    static double[] toRed(double[] pose, Alliance alliance) {
        return forAlliance(pose, alliance);
    }

    static double wallX() {
        return -(HALF_FIELD - FieldAim.ROBOT_HALF_LENGTH);
    }

    /** CELL that is up at the start: our left one, next to the LOADING ZONE (RED: far). {x, y}. */
    static double[] startUpCell() {
        return new double[] {-FieldAim.HIVE_X, FieldAim.CELL_Y};
    }

    /** CELL that is down at the start: our right one, near the GARDEN (RED: audience). {x, y}. */
    static double[] startDownCell() {
        return new double[] {-FieldAim.HIVE_X, -FieldAim.CELL_Y};
    }

    static int startUpCluster(Alliance alliance) {
        return alliance == Alliance.BLUE ? 2 : 0;
    }

    static int startDownCluster(Alliance alliance) {
        return alliance == Alliance.BLUE ? 3 : 1;
    }

    /** Op mode 1: back on the alliance wall, facing the HIVE, lined up with the upward CELL. */
    static double[] startAtUpCell() {
        return new double[] {wallX(), startUpCell()[1], 0};
    }

    /** Op modes 2 and 3: same, lined up with the downward CELL. */
    static double[] startAtDownCell() {
        return new double[] {wallX(), startDownCell()[1], 0};
    }

    static double[] leaveSpot(double[] start) {
        return new double[] {start[0] + LEAVE_IN, start[1], 0};
    }

    static double[] park() {
        return new double[] {PARK_X, PARK_Y, 0};
    }

    static double gardenY() {
        return -(HALF_FIELD - ROBOT_HALF_WIDTH - GARDEN_WALL_GAP_IN);
    }

    /** Lined up along the audience wall, intake (front) toward our corner. */
    static double[] gardenStart() {
        return new double[] {GARDEN_START_X, gardenY(), 180};
    }

    /** Front of the robot GARDEN_WALL_GAP_IN from our alliance wall: all 4 POLLEN passed. */
    static double[] gardenEnd() {
        return new double[] {-(HALF_FIELD - FieldAim.ROBOT_HALF_LENGTH - GARDEN_WALL_GAP_IN), gardenY(), 180};
    }

    /** SHOT_DISTANCE_IN from the CELL at {x, y}, in the direction bearingDeg, facing the HIVE. */
    static double[] shotSpot(double[] cell, double bearingDeg) {
        double b = Math.toRadians(bearingDeg);
        return new double[] {cell[0] + SHOT_DISTANCE_IN * Math.cos(b), cell[1] + SHOT_DISTANCE_IN * Math.sin(b), 0};
    }

    /** After the GARDEN: shoot into the CELL that was down at the start (it was tipped up), from near the GARDEN. */
    static double[] gardenShot() {
        return shotSpot(startDownCell(), GARDEN_SHOT_BEARING_DEG);
    }

    /** Fallback: shoot the preloads into the CELL that was up at the start. */
    static double[] fallbackShot() {
        return shotSpot(startUpCell(), FALLBACK_SHOT_BEARING_DEG);
    }

    /** Fallback route from the downward-CELL start: forward under the HIVE, then right to the shot. */
    static double[][] fallbackRoute() {
        double[] spot = fallbackShot();
        return new double[][] {{spot[0], startAtDownCell()[1], 0}, spot};
    }

    /**
     * Route (RED frame) from a RED pose to the park spot: straight there when close, otherwise
     * out to the lane, along it, then into the LOADING ZONE. All but the last point are pass-through.
     */
    static double[][] parkRoute(double x, double y) {
        double[] park = park();
        if (Math.hypot(park[0] - x, park[1] - y) < 15) return new double[][] {park};
        return new double[][] {{LANE_X, y, 0}, {LANE_X, park[1], 0}, park};
    }
}
