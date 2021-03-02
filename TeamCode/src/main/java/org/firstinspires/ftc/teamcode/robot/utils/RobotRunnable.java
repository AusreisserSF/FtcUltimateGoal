package org.firstinspires.ftc.teamcode.robot.utils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.robot.LCHSRobot;

import java.util.concurrent.atomic.AtomicBoolean;

public abstract class RobotRunnable implements Runnable {

    private static final long SLEEP_INTERVAL = 10;

    private Thread thread;
    private final AtomicBoolean running = new AtomicBoolean(false);

    protected String tag = "Action";
    protected LinearOpMode opMode;

    protected abstract void onRun();
    protected abstract void insideRun() throws SomethingBadHappened;
    protected abstract void onEndRun() throws SomethingBadHappened;
    protected abstract boolean runIsComplete();

    public void start() {
        internalStart();
        thread = new Thread(this);
        thread.start();
    }

    @Override
    public synchronized void run() {
        onRun();

        do {
            try {
                insideRun();
            } catch (SomethingBadHappened x) {
                opMode.requestOpModeStop();
            }

            try {
                Thread.sleep(SLEEP_INTERVAL);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                RobotLog.dd(tag, "Action thread was interrupted");
                break;
            }
        } while (opModeIsActive() && isRunning() && !runIsComplete());

        internalStop();
//        notify();
        RobotLog.dd(tag, "Completed");
    }

    public void stop() {
        RobotLog.dd(tag, "Stop");
        thread.interrupt();
    }

    private void internalStart() {
        running.set(true);
    }

    private void internalStop() {
        running.set(false);
        try {
            onEndRun();
        } catch (SomethingBadHappened x) {
            opMode.requestOpModeStop();
        }
    }

    public boolean isRunning() {
        return running.get();
    }

    private boolean opModeIsActive() {
        if (opMode == null) {
            opMode = LCHSRobot.getInstance().opMode;
        }
        return !opMode.isStopRequested();
    }


}
