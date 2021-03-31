package org.firstinspires.ftc.teamcode.auto.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.CommonUtils;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;

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
    private final T265Camera t265Camera;
    private boolean readerActivated = false;

    // Thread-related.
    private final CountDownLatch countDownLatch = new CountDownLatch(1);
    private T265ReaderCallable t265ReaderCallable;
    private CompletableFuture<Void> readerFuture;
    private final Lock readerLock = new ReentrantLock();
    private final Condition readerCondition = readerLock.newCondition();
    private boolean readerInformationAvailable = false;

    private T265Camera.CameraUpdate latestPose; // shared with the reader thread; protected by the lock

    public T265Reader(LinearOpMode pLinearOpMode, T265Camera pT265Camera) {
        super();
        linearOpMode = pLinearOpMode;
        t265Camera = pT265Camera;
    }

    // Turn on T265 localization.
    public synchronized void activateT265Localization() throws InterruptedException {
        if (readerActivated)
            return; // nothing to do
        readerActivated = true;

        // Start a thread that accesses localization data from the T265.
        readerInformationAvailable = false; // make sure the synchronization flag starts off false
        latestPose = null;

        // Start up the T265 reader as a CompletableFuture.
        RobotLogCommon.d(TAG, "Starting T265 reader thread");

        t265Camera.start();
        t265ReaderCallable = new T265ReaderCallable(countDownLatch);
        readerFuture = CommonUtils.launchAsync(t265ReaderCallable);

        // Wait here until the thread is off and running.
        countDownLatch.await();
    }

    // Turn off when done with T265 localization.
    public synchronized void deactivateT265Localization() throws IOException, InterruptedException {
        if (!readerActivated)
            return; // nothing to do
        readerActivated = false;

        // Stop the reader and wait for the reader to complete.        
        t265Camera.stop();
        t265ReaderCallable.stopThread();
        CommonUtils.getFutureCompletion(readerFuture);
    }

    // Returns the most recently read data from the T265 or an empty Optional
    // if no data is available and the timer expires.
    public Optional<T265Camera.CameraUpdate> getT265Pose(int pTimeout) throws InterruptedException {

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
                        latestPose = update;
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