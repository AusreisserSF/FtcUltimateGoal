package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.*;
import org.opencv.core.Mat;
import java.util.Date;

public class FileImage implements ImageProvider {

    private final String pathToImageFile;
    private final ImageUtils imageUtils = new ImageUtils();

    public FileImage(String pPathToImageFile) {
        pathToImageFile = pPathToImageFile;
    }

    @Override
    // LocalDateTime requires minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
    public Pair<Mat, Date> getImage() {
        Mat bgrMat = imageUtils.loadImage(pathToImageFile);    // LocalDateTime requires minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        return Pair.create(bgrMat, new Date());
    }

    @Override
    public ImageFormat getImageFormat() {
        return ImageFormat.BGR;
    }

}

