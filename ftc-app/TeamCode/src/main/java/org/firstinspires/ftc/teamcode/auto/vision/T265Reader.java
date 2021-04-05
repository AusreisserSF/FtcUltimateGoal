package org.firstinspires.ftc.teamcode.auto.vision;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.CommonUtils;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.teamcode.math.Pose;

import java.io.IOException;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

// Class that continuously reads localization data from a T265 camera and posts the results.
public class T265Reader {

    private static final String TAG = "T265Reader";

    // For reading localization data from an Intel RealSense T265 camera.
    private final LinearOpMode linearOpMode;
    private final HardwareMap hardwareMap;
    private T265Camera t265Camera;
    private boolean readerActivated = false;

    // Thread-related.
    private final CountDownLatch countDownLatch = new CountDownLatch(1);
    private T265ReaderCallable t265ReaderCallable;
    private CompletableFuture<Void> readerFuture;
    private final Lock readerLock = new ReentrantLock();
    private final Condition readerCondition = readerLock.newCondition();
    private boolean readerInformationAvailable = false;

    private Pose latestPose; // shared with the reader thread; protected by the lock

    public T265Reader(HardwareMap pHardwareMap, LinearOpMode pLinearOpMode) {
        super();
        hardwareMap = pHardwareMap;
        linearOpMode = pLinearOpMode;
    }

    // Turn on T265 localization. Returns false if the T265 device could not be
    // initialized.
    public synchronized boolean activateT265Localization() throws InterruptedException {
        if (readerActivated)
            return true;
        readerActivated = true;

        Transform2d cameraToRobot = new Transform2d();
        double encoderMeasurementCovariance = 0.8; // Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
        Pose2d startingPose = new Pose2d(0, 0, new Rotation2d());

        t265Camera = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);
        if (t265Camera == null)
            return false; // nothing to do

        t265Camera.setPose(startingPose);
        t265Camera.start(); // start the camera stream
        readerInformationAvailable = false; // make sure the synchronization flag starts off false
        latestPose = null;

        // Start up the T265 reader as a CompletableFuture.
        RobotLogCommon.d(TAG, "Starting T265 reader thread");

        t265ReaderCallable = new T265ReaderCallable(countDownLatch);
        readerFuture = CommonUtils.launchAsync(t265ReaderCallable);

        // Wait here until the thread is off and running.
        countDownLatch.await();
        return true;
    }

    // Turn off when done with T265 localization.
    public synchronized void deactivateT265Localization() throws IOException, InterruptedException {
        if (!readerActivated)
            return; // nothing to do
        readerActivated = false;

        // Stop the reader and wait for it to complete.
        t265ReaderCallable.stopThread();
        t265Camera.stop();
        t265Camera = null;
        CommonUtils.getFutureCompletion(readerFuture);
    }

    // Returns the most recently read data from the T265 or an empty Optional
    // if no data is available and the timer expires.
    public Optional<Pose> getT265Pose(int pTimeout) throws InterruptedException {

        RobotLogCommon.d(TAG, "Looking for T265 data with timeout value " + pTimeout + " ms");

        readerLock.lock();
        if (!readerActivated)
            throw new AutonomousRobotException(TAG, "getT265Pose(): T265 is not activated");

        try {
            boolean waitVal = true;
            long now = System.currentTimeMillis();
            long deadline = now + pTimeout;
            while (!readerInformationAvailable && (now < deadline)) {
                waitVal = readerCondition.await(deadline - now, TimeUnit.MILLISECONDS);
                if (!waitVal)
                    break; // timed out
                now = System.currentTimeMillis();
            }

            if (!waitVal || (now >= deadline)) {
                // RobotLogCommon.d(TAG, "Timed out waiting for T265 data " + pTimeout + " ms");
                return Optional.empty();
            }

            // Return the data from the T265.
            return Optional.of(latestPose);

        } finally {
            latestPose = null;
            readerInformationAvailable = false;
            readerLock.unlock();
            // RobotLogCommon.d(TAG, "Exiting getT265Pose");
        }
    }

    // Looks for T265 data and, if present, makes it available to the main thread.
    private class T265ReaderCallable extends AutoWorker<Void> {

        T265ReaderCallable(CountDownLatch pCountDownLatch) {
            super(pCountDownLatch);
        }

        public Void call() {
            RobotLogCommon.d(TAG, "In T265Reader thread");
            countDownLatch.countDown(); // signal that I'm running

            while (linearOpMode.opModeIsActive() && !stopThreadRequested()) {
                T265Camera.CameraUpdate update = t265Camera.getLastReceivedCameraUpdate();
                if (update != null) {
                    // RobotLogCommon.v(TAG, "Got pose information from the T265");
                    readerLock.lock();
                    try {
                        readerInformationAvailable = true;

                        latestPose = new Pose(
                                update.pose.getTranslation().getX(),
                                update.pose.getTranslation().getY(),
                                update.pose.getRotation().getDegrees()
                        );

                        readerCondition.signal(); // let the main thread know
                    } finally {
                        readerLock.unlock();
                    }
                    break;
                }
            }

            return null;
        }
    }

}