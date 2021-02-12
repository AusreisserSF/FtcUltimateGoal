package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.opencv.core.Mat;
import java.util.Date;

public interface ImageProvider {

    enum ImageFormat {RGB, BGR}

    // LocalDateTime requires minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
    Pair<Mat, Date> getImage() throws InterruptedException;

    ImageFormat getImageFormat();
}
