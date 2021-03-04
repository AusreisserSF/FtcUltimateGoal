package org.firstinspires.ftc.teamcode.auto.vision;

import android.graphics.Bitmap;

import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.Frame;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.Date;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;

public class VuforiaImage implements ImageProvider {

    private static final String TAG = "LCHSVuforiaImage";
    private final VuforiaLocalizer autoVuforia;
    private Bitmap bitmap;
    private CountDownLatch frameAcquired;

    public VuforiaImage(VuforiaLocalizer pVuforia) {
        autoVuforia = pVuforia;
    }

    // Warning - may return a pair of null,null if no frame was available in 5 tries.
    // LocalDateTime requires minSdkVersion 26  public Pair<Mat, LocalDateTime> getImage() throws InterruptedException;
    @Override
   public Pair<Mat, Date> getImage() throws InterruptedException {
        Bitmap vuforiaFrame;

        for (int i = 0; i < 5; i++) {
            vuforiaFrame = captureVuforiaFrame();
            if (vuforiaFrame != null) {
                // Convert the bitmap returned by Vuforia into an OpenCV Mat for image processing.
                Mat matFromBitmap = new Mat();
                Utils.bitmapToMat(vuforiaFrame, matFromBitmap); // outputs an RGB image
                return Pair.create(matFromBitmap, new Date());
            }
            android.os.SystemClock.sleep(100);
        }

        RobotLogCommon.d(TAG, "Got 5 null frames from Vuforia");
        return null;
    }

    @Override
    public ImageFormat getImageFormat() {
        return ImageFormat.RGB;
    }

    // Modified from ConceptVuforiaNavigationWebcam.
    /**
     * Sample one frame from the Vuforia stream.
     */
    // Warning, the returned Bitmap may be null. The caller must be prepared
    // to retry if desired.
    private Bitmap captureVuforiaFrame() throws InterruptedException {
        // We need frame capture to be synchronous so prepare to wait here.
        frameAcquired = new CountDownLatch(1);
        autoVuforia.getFrameOnce(Continuation.create(ThreadPool.getDefault(), (Frame frame) -> {
            bitmap = autoVuforia.convertFrameToBitmap(frame);
            if (bitmap == null)
                RobotLogCommon.d(TAG, "Vuforia returned a null Bitmap");
            else
                RobotLogCommon.d(TAG, "Got an image from Vuforia");
            frameAcquired.countDown();
        }));

        // Wait here until Vuforia gives us a frame or the timer expires.
        if (!frameAcquired.await(1500, TimeUnit.MILLISECONDS)) // returns false if timer expires
            throw new AutonomousRobotException(TAG, "Timed out waiting for a frame from Vuforia");
        return bitmap;
    }
}
