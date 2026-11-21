package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.TurretController.Alliance;

/**
 * Field spots for the autonomous OpModes (BIOBUZZ Competition Manual TU10, sections 9 and 10).
 *
 * Every spot is written for RED, as {x, y, headingDeg} in the FieldAim frame (origin at the field
 * centre, +X toward the blue wall, +Y away from the audience, heading counter-clockwise, 0 = facing
 * the blue wall, 90 = facing away from the audience). BLUE's field is RED's turned 180 degrees about
 * the centre, so forAlliance() mirrors every spot (RED C1 = BLUE D6, A5 = F2, ...).
 *
 * Tiles (Fig 9-5): columns A..F from x = -72 (red wall) in 24 in. steps, rows 1..6 from y = -72
 * (audience wall). RED's field, as on the team's drawing (2026-11-21):
 *   - HIVE CELLs in column C: C3 (audience side) is UP at the start, C4 (far side) is down.
 *   - LOADING ZONE in A5: x -72..-61, y +24..+48.
 *   - GARDEN in A1: 4 POLLEN in a row along the audience wall, x -71..-62, y about -70.6.
 *   - Auto 1 starts in C1 (back on the audience wall), Auto 2/3 in C6 (back on the far wall),
 *     both facing the HIVE and lined up with the CELL in front of them.
 */
public final class AutoField {
    public static double ROBOT_HALF_WIDTH = 9.0; // robot centre to its side (18 in. robot)
    // Robot centre this far forward from its start spot after "leave".
    public static double LEAVE_IN = 4.0;
    // Lane the robot drives up and down the field in, clear of the HIVE legs (x about -25).
    public static double LANE_X = -40.0;
    // Park: robot centre in A5; its left part then overlaps the LOADING ZONE (x up to -61).
    public static double PARK_X = -58.0, PARK_Y = 36.0;
    // GARDEN pickup: drive along the audience wall, intake first, from GARDEN_START_X to the corner.
    public static double GARDEN_START_X = -40.0;
    public static double GARDEN_WALL_GAP_IN = 1.0; // gap between the robot's side and the wall
    // 1.2 m shots. Bearing = direction from the CELL to the robot, degrees counter-clockwise.
    public static double SHOT_DISTANCE_IN = 47.2;
    // After the GARDEN: shoot into C4 from next to the LOADING ZONE, then park right there.
    public static double GARDEN_SHOT_BEARING_DEG = 160.0;
    // Fallback (Auto 2, not tipped): from C6 drive straight under the HIVE down to FALLBACK_UNDER_Y,
    // then to the right (toward our wall) and shoot diagonally into C3. Stops short of the
    // partner's spot in C1, and stays on our half (G402).
    public static double FALLBACK_UNDER_Y = -35.0;
    public static double FALLBACK_SHOT_BEARING_DEG = 225.0;

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

    /** CELL that is up at the start (RED: C3, audience side). {x, y}. */
    static double[] startUpCell() {
        return new double[] {-FieldAim.HIVE_X, -FieldAim.CELL_Y};
    }

    /** CELL that is down at the start (RED: C4, far side). {x, y}. */
    static double[] startDownCell() {
        return new double[] {-FieldAim.HIVE_X, FieldAim.CELL_Y};
    }

    static int startUpCluster(Alliance alliance) {
        return alliance == Alliance.BLUE ? 3 : 1;
    }

    static int startDownCluster(Alliance alliance) {
        return alliance == Alliance.BLUE ? 2 : 0;
    }

    /** Auto 1 (RED C1): back on the audience wall, facing the HIVE, lined up with the upward CELL. */
    static double[] startAtUpCell() {
        return new double[] {startUpCell()[0], -(HALF_FIELD - FieldAim.ROBOT_HALF_LENGTH), 90};
    }

    /** Auto 2 and 3 (RED C6): back on the far wall, facing the HIVE, lined up with the downward CELL. */
    static double[] startAtDownCell() {
        return new double[] {startDownCell()[0], HALF_FIELD - FieldAim.ROBOT_HALF_LENGTH, -90};
    }

    /** LEAVE_IN straight ahead of a start pose. */
    static double[] leaveSpot(double[] start) {
        double h = Math.toRadians(start[2]);
        return new double[] {start[0] + LEAVE_IN * Math.cos(h), start[1] + LEAVE_IN * Math.sin(h), start[2]};
    }

    static double[] park(double headingDeg) {
        return new double[] {PARK_X, PARK_Y, headingDeg};
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

    /** SHOT_DISTANCE_IN from the CELL at {x, y}, in the direction bearingDeg, facing the blue wall. */
    static double[] shotSpot(double[] cell, double bearingDeg) {
        double b = Math.toRadians(bearingDeg);
        return new double[] {cell[0] + SHOT_DISTANCE_IN * Math.cos(b), cell[1] + SHOT_DISTANCE_IN * Math.sin(b), 0};
    }

    /** After the GARDEN: shoot into the CELL that was down at the start (it was tipped up). */
    static double[] gardenShot() {
        return shotSpot(startDownCell(), GARDEN_SHOT_BEARING_DEG);
    }

    /** Fallback: shoot the preloads into the CELL that was up at the start. */
    static double[] fallbackShot() {
        return shotSpot(startUpCell(), FALLBACK_SHOT_BEARING_DEG);
    }

    /** From the C6 start along the far wall to the lane, down the lane, then along the audience wall. */
    static double[][] gardenRoute() {
        double[] start = startAtDownCell();
        return new double[][] {
            {LANE_X, start[1], start[2]},
            {LANE_X, gardenY() + 10, 180},
            gardenStart()};
    }

    /** From the GARDEN corner back up the lane to the 1.2 m shot next to the LOADING ZONE. */
    static double[][] gardenShotRoute() {
        double[] shot = gardenShot();
        return new double[][] {{LANE_X, gardenY() + 7, 0}, {LANE_X, shot[1], 0}, shot};
    }

    /** Fallback from the C6 start: straight ahead under the HIVE, then right to the shot. */
    static double[][] fallbackRoute() {
        double[] start = startAtDownCell();
        return new double[][] {{start[0], FALLBACK_UNDER_Y, start[2]}, fallbackShot()};
    }

    /**
     * Route (RED frame) from a RED pose to the park spot: straight there when close, otherwise
     * out to the lane, along it, then into the LOADING ZONE, keeping the current heading.
     * All but the last point are pass-through.
     */
    static double[][] parkRoute(double x, double y, double headingDeg) {
        double[] park = park(headingDeg);
        if (Math.hypot(park[0] - x, park[1] - y) < 15) return new double[][] {park};
        return new double[][] {{LANE_X, y, headingDeg}, {LANE_X, park[1], headingDeg}, park};
    }
}
