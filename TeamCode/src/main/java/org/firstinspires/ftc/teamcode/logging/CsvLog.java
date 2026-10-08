package org.firstinspires.ftc.teamcode.logging;

import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.Locale;

/**
 * Writes one CSV file per OpMode run to the Robot Controller, in /sdcard/logs/, so a run can be replayed
 * and analyzed afterward. Pull the files off with: adb pull /sdcard/logs/
 * No hardware. A file problem never stops the OpMode: if the file can't be written, logging just turns off,
 * getErrorMessage() says why, and the error is also written to the robot's logcat (tag CsvLog), which ends up in
 * the SDK's own /sdcard/FIRST/matchlogs/ file for the run.
 */
public class CsvLog {
    // Tag for messages in the robot's logcat, so they can be found with: adb logcat -s CsvLog
    private static final String LOGCAT_TAG = "CsvLog";
    // Rows written between flushes to the file. Flushing every row would slow the loop; flushing rarely risks
    // losing the end of the run if the robot loses power before close()
    private static final int ROWS_PER_FLUSH = 25;

    private final File logFile;
    private BufferedWriter fileWriter; // null once logging has turned off (closed, or a write failed)
    private final int columnCount;
    private int rowsSinceFlush = 0;
    private String errorMessage = null;

    /**
     * Creates the log file and writes the header row. The file name is the OpMode name plus the date and time,
     * e.g. MatchAuto_2026-10-07_14-03-22.csv, so runs never overwrite each other.
     *
     * @param opModeName  used at the start of the file name
     * @param columnNames the header row; every later addRow() must pass one value per column, in this order
     */
    public CsvLog(String opModeName, String... columnNames) {
        columnCount = columnNames.length;
        String timestamp = new SimpleDateFormat("yyyy-MM-dd_HH-mm-ss", Locale.US).format(new Date());
        File logFolder = new File(AppUtil.ROOT_FOLDER, "logs"); // ROOT_FOLDER is /sdcard (not /sdcard/FIRST)
        logFile = new File(logFolder, opModeName + "_" + timestamp + ".csv");
        try {
            //noinspection ResultOfMethodCallIgnored -- an existing folder is fine; a real failure shows up on the next line
            logFolder.mkdirs();
            fileWriter = new BufferedWriter(new FileWriter(logFile));
            writeLine(columnNames);
            RobotLog.ii(LOGCAT_TAG, "Logging to " + logFile.getAbsolutePath());
        } catch (IOException exception) {
            turnOffAfterError(exception);
        }
    }

    /**
     * Adds one row. Values are written with String.valueOf(); text containing commas, quotes or line breaks
     * is quoted so the CSV stays readable.
     *
     * @param values one value per column, in header order
     */
    public void addRow(Object... values) {
        if (fileWriter == null) {
            return;
        }
        if (values.length != columnCount) {
            errorMessage = "Row has " + values.length + " values but the header has " + columnCount + " columns";
            RobotLog.ee(LOGCAT_TAG, errorMessage);
            return; // skip the bad row instead of writing a CSV that no longer lines up with its header
        }
        try {
            writeLine(values);
            rowsSinceFlush++;
            if (rowsSinceFlush >= ROWS_PER_FLUSH) {
                fileWriter.flush();
                rowsSinceFlush = 0;
            }
        } catch (IOException exception) {
            turnOffAfterError(exception);
        }
    }

    /**
     * Writes everything still buffered and closes the file. Call from the OpMode's stop(). Safe to call twice.
     */
    public void close() {
        if (fileWriter == null) {
            return;
        }
        try {
            fileWriter.close(); // close() flushes first
        } catch (IOException exception) {
            errorMessage = exception.getMessage();
        }
        fileWriter = null;
    }

    /**
     * Where the log is being written, for showing on telemetry.
     *
     * @return the full path of the log file
     */
    public String getFilePath() {
        return logFile.getAbsolutePath();
    }

    /**
     * Why logging failed or a row was skipped, if it did.
     *
     * @return the last error, or null if there hasn't been one
     */
    public String getErrorMessage() {
        return errorMessage;
    }

    /**
     * Writes one CSV line made of the given values.
     *
     * @param values the cells of the line
     * @throws IOException if the file can't be written
     */
    private void writeLine(Object[] values) throws IOException {
        StringBuilder line = new StringBuilder();
        for (int columnIndex = 0; columnIndex < values.length; columnIndex++) {
            if (columnIndex > 0) {
                line.append(',');
            }
            line.append(escapeCell(String.valueOf(values[columnIndex])));
        }
        fileWriter.write(line.toString());
        fileWriter.newLine();
    }

    /**
     * Quotes a cell if it contains anything that would break the CSV, doubling any quotes inside it
     * (the standard CSV way to write a literal quote).
     *
     * @param cellText the cell's text
     * @return the text, quoted if needed
     */
    private static String escapeCell(String cellText) {
        boolean needsQuotes = cellText.indexOf(',') >= 0 || cellText.indexOf('"') >= 0
                || cellText.indexOf('\n') >= 0 || cellText.indexOf('\r') >= 0;
        if (!needsQuotes) {
            return cellText;
        }
        return '"' + cellText.replace("\"", "\"\"") + '"';
    }

    /**
     * Records the error and stops logging, so a broken file never stops the OpMode.
     *
     * @param exception what went wrong
     */
    private void turnOffAfterError(IOException exception) {
        errorMessage = exception.getMessage();
        RobotLog.ee(LOGCAT_TAG, exception, "Logging to " + logFile.getAbsolutePath() + " turned off");
        if (fileWriter != null) {
            try {
                fileWriter.close();
            } catch (IOException ignored) {
                // Already failing; nothing more useful to do with a second error
            }
        }
        fileWriter = null;
    }
}
