package org.firstinspires.ftc.teamcode;

/**
 * Flywheel speed and ball flight time for a horizontal distance to the goal.
 *
 * Fill ROWS from the "BIOBUZZ Shot Calibration" OpMode: stand at 4-6 distances, adjust the RPM
 * until shots go in, and copy the row it shows. Keep rows sorted by distance. Between rows the
 * values are interpolated; outside them auto-fire is refused (with INRANGE_MARGIN_IN slack).
 *
 * Flight time is only used to lead moving shots. Start from a slow-motion video of a shot, then
 * tune with drive-bys (or use TurretController.LEAD_GAIN for an overall correction).
 */
public final class ShotTable {
    // {horizontal distance (in), flywheel RPM (measured), flight time (s)}
    // Measured on the mock CELL (tags 123 cm high), 2026-11-08. Below ~45 in the camera loses
    // the tags at its 35 deg tilt. Flight times are guesses until measured on video.
    public static double[][] ROWS = {
        {42.0, 1850.0, 0.40},
        {58.0, 2000.0, 0.45},
        {70.0, 2150.0, 0.50},
        {80.0, 2300.0, 0.55},
    };
    // How far beyond the first/last row auto-fire is still allowed.
    public static double INRANGE_MARGIN_IN = 6.0;

    static final int RPM = 1;
    static final int FLIGHT_TIME = 2;

    private ShotTable() {
    }

    public static double rpm(double distanceIn) {
        return interpolate(ROWS, RPM, distanceIn);
    }

    public static double flightTime(double distanceIn) {
        return interpolate(ROWS, FLIGHT_TIME, distanceIn);
    }

    public static boolean inRange(double distanceIn) {
        return inRange(ROWS, distanceIn);
    }

    static double flightTime(double[][] rows, double distanceIn) {
        return interpolate(rows, FLIGHT_TIME, distanceIn);
    }

    static boolean inRange(double[][] rows, double distanceIn) {
        if (rows.length == 0 || distanceIn <= 0) return false;
        return distanceIn >= rows[0][0] - INRANGE_MARGIN_IN
            && distanceIn <= rows[rows.length - 1][0] + INRANGE_MARGIN_IN;
    }

    /** Linear interpolation of a column by distance, held flat beyond the first and last rows. */
    static double interpolate(double[][] rows, int column, double distanceIn) {
        if (rows.length == 0) return 0.0;
        if (distanceIn <= rows[0][0]) return rows[0][column];
        for (int i = 1; i < rows.length; i++) {
            if (distanceIn <= rows[i][0]) {
                double[] a = rows[i - 1], b = rows[i];
                double f = (distanceIn - a[0]) / (b[0] - a[0]);
                return a[column] + f * (b[column] - a[column]);
            }
        }
        return rows[rows.length - 1][column];
    }
}
