package org.firstinspires.ftc.ftcdevcommon.android;

import android.os.Environment;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;

import java.io.File;
import java.io.IOException;

public class WorkingDirectory {
    private static final String TAG = "WorkingDirectory";
    private static String picturesPath;
    private static String ioExceptionMessage;

    static {
        File picturesDir = Environment.getExternalStoragePublicDirectory(Environment.DIRECTORY_PICTURES);
        try {
            picturesPath = picturesDir.getCanonicalPath();
        } catch (IOException iox) {
            // Can't do anything with the exception here but see
            // getWorkingDirectory below.
            ioExceptionMessage = iox.getMessage();
        }
    }

    public static String getWorkingDirectory() {
        if (picturesPath == null)
            throw new AutonomousRobotException(TAG, "Could not open working directory " + ioExceptionMessage);
        return picturesPath;
    }
}
