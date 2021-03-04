// Partial port of ImageUtils.cpp: only thos functions that are
// needed for ring recognition.
package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.opencv.core.Mat;
import org.opencv.core.Core;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.core.Rect;

import java.util.ArrayList;
import java.util.Arrays;

import static org.opencv.imgcodecs.Imgcodecs.IMREAD_COLOR;

public class ImageUtils {

    public static final String TAG = "ImageUtils";

    // Load an image.
    public Mat loadImage(String pInputFilename) {
        RobotLogCommon.d(TAG, "File name " + pInputFilename);
        Mat imageOut = Imgcodecs.imread(pInputFilename, IMREAD_COLOR);
        if (imageOut.empty())
            throw new AutonomousRobotException(TAG, "Could not find or open the image " + pInputFilename);

        RobotLogCommon.d(TAG, "Image width " + imageOut.cols() + ", height " + imageOut.rows());
        return imageOut;
    }

    // Define a region of interest.
    public Mat getImageROI(Mat pSrcImage, Rect pROIDefinition) {

        if ((pROIDefinition.height == 0) && (pROIDefinition.width == 0)) {
            RobotLogCommon.d(TAG, "At least one ROI dimension was 0");
            return new Mat();
        }

        Mat roi = new Mat(pSrcImage, pROIDefinition);
        RobotLogCommon.d(TAG, "Image ROI x " + pROIDefinition.x + ", y " + pROIDefinition.y + ", width " + pROIDefinition.width + ", height " + pROIDefinition.height);
        return roi;
    }

    // Adjust image saturation and value levels in the image to match the targets.
    public Mat adjustSaturationAndValue(Mat pHSVImage, int pSatLowTarget, int pValLowTarget) {
        // Split the image into its constituent HSV channels
        ArrayList<Mat> channels = new ArrayList<>();
        Core.split(pHSVImage, channels);

        // Get the median of the S channel.
        int medianSaturation = getColorChannelMedian(channels.get(1), new Mat());

        // Get the median of the V channel.
        int medianValue = getColorChannelMedian(channels.get(2), new Mat());

        RobotLogCommon.d(TAG, "HSV saturation channel median " + medianSaturation);
        RobotLogCommon.d(TAG, "HSV value channel median " + medianValue);

        // adjustment = target - median;
        int satAdjustment = pSatLowTarget - medianSaturation;
        int valAdjustment = pValLowTarget - medianValue;
        channels.get(1).convertTo(channels.get(1), -1, 1, satAdjustment);
        channels.get(2).convertTo(channels.get(2), -1, 1, valAdjustment);

        RobotLogCommon.d(TAG, "Adjust HSV saturation by " + satAdjustment);
        RobotLogCommon.d(TAG, "Adjust HSV value by " + valAdjustment);

        // Merge the channels back together.
        Mat adjustedImage = new Mat();
        Core.merge(channels, adjustedImage);
        return adjustedImage;
    }

    // Get the median of any single-channel Mat.
    public int getSingleChannelMedian(Mat pSingleChannelMat) {

        if ((pSingleChannelMat.dims() != 2) || (!pSingleChannelMat.isContinuous()))
            throw new AutonomousRobotException(TAG, "Expected a single-channel Mat");

        byte[] byteBuff = new byte[(int) pSingleChannelMat.total()];
        int[] intBuff = new int[(int) pSingleChannelMat.total()];
        int buffLength = byteBuff.length;
        pSingleChannelMat.get(0, 0, byteBuff);

        // !! Since Java does not have an unsigned char data type, the byte values
        // may come out as negative. So we have to use a separate array of ints and
        // copy in the bytes with the lower 8 bytes preserved.
        // https://stackoverflow.com/questions/9581530/converting-from-byte-to-int-in-java
        for (int i = 0; i < buffLength; i++)
            intBuff[i] = Byte.toUnsignedInt(byteBuff[i]); // or byteBuff[i] & 0xFF;

        Arrays.sort(intBuff);
        int median = (intBuff[buffLength / 2] + (intBuff[(buffLength / 2) - 1])) / 2;
        return median;
    }

    // Get the median of a color channel.
    private int getColorChannelMedian(Mat pChannel, Mat pMask) {
        // If we're dealing with a non-masked image then we just take the median
        // of all the pixels.
        if (pMask.total() == 0) {
            return getSingleChannelMedian(pChannel);
        } else
            throw new AutonomousRobotException(TAG, "getColorChannelMedian with mask is not supported at this time");
    }

}

