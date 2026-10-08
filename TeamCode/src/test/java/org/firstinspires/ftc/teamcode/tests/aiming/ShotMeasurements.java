package org.firstinspires.ftc.teamcode.tests.aiming;

/**
 * The raw shot measurements the shot table was built from, shared by the tests. All taken against the measured
 * cell (the blue hive's lower-right cell, ShotTarget.BLUE_LOWER).
 * Each row: x (in), y (in), measured heading (deg), min working RPM, max working RPM.
 * x and y are as recorded, in the old TeleOp frame; use pedroX() / pedroY() to get Pedro field coordinates.
 */
final class ShotMeasurements {
    static final double[][] ROWS = {
            {73.5, 2.88, 270.5, 2350, 2500},
            {44.89, 11.6, 228.3, 2300, 2450},
            {26.15, 6.34, 226.18, 2450, 2650},
            {38.697, 9.1367, 224.1, 2350, 2500},
            {52.21, 8.05, 245.58, 2300, 2450},
            {73.5, 5.6, 270, 2450, 2500}, // closest workable point (~42 in)
            {73.5, 0, 270, 2400, 2500},
    };

    // Column indexes into each row
    static final int X = 0;
    static final int Y = 1;
    static final int HEADING_DEG = 2;
    static final int MIN_RPM = 3;
    static final int MAX_RPM = 4;

    // The old TeleOp frame is Pedro minus this much on both x and y
    static final double OLD_FRAME_TO_PEDRO_OFFSET_INCHES = 8.5;

    /**
     * Not meant to be instantiated; this class only holds data.
     */
    private ShotMeasurements() {
    }

    /**
     * A row's x in Pedro field coordinates.
     *
     * @param row a measurement row
     * @return the row's x converted from the old TeleOp frame to Pedro
     */
    static double pedroX(double[] row) {
        return row[X] + OLD_FRAME_TO_PEDRO_OFFSET_INCHES;
    }

    /**
     * A row's y in Pedro field coordinates.
     *
     * @param row a measurement row
     * @return the row's y converted from the old TeleOp frame to Pedro
     */
    static double pedroY(double[] row) {
        return row[Y] + OLD_FRAME_TO_PEDRO_OFFSET_INCHES;
    }

    /**
     * Describes a row for assertion messages.
     *
     * @param row a measurement row
     * @return the row's position as "(x, y)"
     */
    static String describe(double[] row) {
        return "(" + row[X] + ", " + row[Y] + ")";
    }
}
