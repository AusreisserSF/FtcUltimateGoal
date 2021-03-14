package org.firstinspires.ftc.teamcode.auto.vision;

//!! Android only

import org.opencv.android.OpenCVLoader;

import org.firstinspires.ftc.ftcdevcommon.CommonUtils;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.RobotConstantsUltimateGoal;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.time.LocalDateTime;
import java.util.Date;

public class RingRecognition {

    private static final String TAG = "RingRecognition";
    private static final String imageFilePrefix = "Image_";

    private final String workingDirectory;
    private final ImageUtils imageUtils;

    public RingRecognition() {
        workingDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.imageDir;
        imageUtils = new ImageUtils();
    }

    // Returns the result of the image analysis.
    public RingReturn findGoldRings(ImageProvider pImageProvider, RingParameters pRingParameters) throws InterruptedException {

        RobotLogCommon.d(TAG, "In RingRecognition.findGoldRings");

        // LocalDateTime requires minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
        Pair<Mat, Date> ringImage = pImageProvider.getImage();
        if (ringImage.first == null)
            return new RingReturn(true, RobotConstantsUltimateGoal.TargetZone.TARGET_ZONE_NPOS); // don't crash

        String fileDate = CommonUtils.getDateTimeStamp(ringImage.second);
        String outputFilenamePreamble = workingDirectory + imageFilePrefix + fileDate;

        // The image may be RGB (from a camera) or BGR ( OpenCV imread from a file).
        Mat imgOriginal = ringImage.first.clone();

        // If you don't convert RGB to BGR here then the _IMG.png file will be written
        // out with incorrect colors (gold will show up as blue).
        if (pImageProvider.getImageFormat() == ImageProvider.ImageFormat.RGB)
            Imgproc.cvtColor(imgOriginal, imgOriginal, Imgproc.COLOR_RGB2BGR);

        String imageFilename = outputFilenamePreamble + "_IMG.png";
        RobotLogCommon.d(TAG, "Writing original image " + imageFilename);
        Imgcodecs.imwrite(imageFilename, imgOriginal);

        // Crop the image to reduce distractions.
        Mat imageROI = imageUtils.getImageROI(imgOriginal, pRingParameters.imageParameters.image_roi);
        imageFilename = outputFilenamePreamble + "_ROI.png";
        RobotLogCommon.d(TAG, "Writing image ROI " + imageFilename);
        Imgcodecs.imwrite(imageFilename, imageROI);

        // Adapted from ...\OpenCV_Projects\OpenCVTestbed2\OpenCVTestbed2\GeneralTarget.cpp
        // We're on the HSV path.
        Mat hsvROI = new Mat();
        Imgproc.cvtColor(imageROI, hsvROI, Imgproc.COLOR_BGR2HSV);

        // Adjust the HSV saturation and value levels in the image to match the targets.
        int goldHueLow = pRingParameters.hsvParameters.hue_low;
        int goldHueHigh = pRingParameters.hsvParameters.hue_high;
        int goldSatTarget = pRingParameters.hsvParameters.saturation_target;
        int goldSatHigh = 255;
        int goldValTarget = pRingParameters.hsvParameters.value_target;
        int goldValHigh = 255;
        RobotLogCommon.d(TAG, "Target hue levels: low " + goldHueLow + ", high " + goldHueHigh);

        // Use inRange to threshold to binary.
        Mat thresholded = new Mat();
        int inRangeSatLow = pRingParameters.hsvParameters.saturation_low_threshold;
        int inrangeValLow = pRingParameters.hsvParameters.value_low_threshold;
        RobotLogCommon.d(TAG, "Actual inRange HSV levels: hue low " + goldHueLow + ", hue high " + goldHueHigh);
        RobotLogCommon.d(TAG, "Actual inRange HSV levels: saturation low " + inRangeSatLow + ", value low " + inrangeValLow);

        Core.inRange(hsvROI, new Scalar(goldHueLow, inRangeSatLow, inrangeValLow), new Scalar(goldHueHigh, goldSatHigh, goldValHigh), thresholded);
        Imgcodecs.imwrite(outputFilenamePreamble + "_HSV_THR.png", thresholded);
        RobotLogCommon.d(TAG, "Writing " + outputFilenamePreamble + "_HSV_THR.png");

        // Instead of trying to find contours, just count the number of white pixels,
        // which are those that are in range for the gold color.
        int white_pixels = Core.countNonZero(thresholded);
        RobotLogCommon.d(TAG, "Number of white pixels " + white_pixels);

        // If the number of white pixels is less than the minimum for a single
        // ring then assume there are no rings on the field.
        RobotConstantsUltimateGoal.TargetZone targetZone;
        if (white_pixels < pRingParameters.minimum_pixel_count_1_ring) {
            targetZone = RobotConstantsUltimateGoal.TargetZone.TARGET_ZONE_A;
            RobotLogCommon.d(TAG, "No rings detected: set Target Zone Goal A");
        } else

            // If the number of white pixels is greater than the minimum for a stack
            // of  4 rings then the target is Goal C.
            if (white_pixels > pRingParameters.minimum_pixel_count_4_rings) {
                targetZone = RobotConstantsUltimateGoal.TargetZone.TARGET_ZONE_C;
                RobotLogCommon.d(TAG, "Found four rings: set Target Zone Goal C");
            } else { // Must be 1 ring.
                targetZone = RobotConstantsUltimateGoal.TargetZone.TARGET_ZONE_B;
                RobotLogCommon.d(TAG, "Found one ring: set Target Zone Goal B");
            }

        return new RingReturn(false, targetZone);
    }

}
