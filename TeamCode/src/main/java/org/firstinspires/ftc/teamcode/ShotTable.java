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
    // Measured 2026-11-20, robot at an angle to the goal ("diagonal"), tape distances in cm
    // converted to inches. Head-on shots agreed within 50 RPM. Flight times are guesses
    // (0.30 s + 3 ms per inch) until measured on video.
    public static double[][] ROWS = {
        {27.6, 1700.0, 0.38}, //  70 cm
        {31.5, 1750.0, 0.39}, //  80 cm
        {35.4, 1775.0, 0.41}, //  90 cm
        {39.4, 1800.0, 0.42}, // 100 cm
        {47.2, 1900.0, 0.44}, // 120 cm
        {59.1, 2000.0, 0.48}, // 150 cm
        {70.9, 2100.0, 0.51}, // 180 cm
        {82.7, 2150.0, 0.55}, // 210 cm
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
