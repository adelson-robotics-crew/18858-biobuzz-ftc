package org.firstinspires.ftc.teamcode.aiming;

/**
 * The raw shot measurements the shot table was built from (our field frame), shared by the tests.
 * Each row: x (in), y (in), measured heading (deg), min working RPM, max working RPM.
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

    /**
     * Not meant to be instantiated; this class only holds data.
     */
    private ShotMeasurements() {
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
