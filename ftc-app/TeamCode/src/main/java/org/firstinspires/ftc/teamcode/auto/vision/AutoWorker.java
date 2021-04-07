package org.firstinspires.ftc.teamcode.auto.vision;

import java.util.concurrent.Callable;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.atomic.AtomicBoolean;

// Base class for long-running threads that supply information,
// e.g. the most recent IMU reading or the most recent webcam
// frame.
public abstract class AutoWorker<T> implements Callable<T> {

    private AtomicBoolean stopThread = new AtomicBoolean();

    public AutoWorker() {}

    @Override
    public abstract T call() throws InterruptedException;

    public void stopThread() {
        stopThread.set(true);
    }

    public boolean stopThreadRequested() {
        return stopThread.get();
    }

}
