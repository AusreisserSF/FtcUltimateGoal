package org.firstinspires.ftc.ftcdevcommon;

import android.util.Log;

import java.util.logging.FileHandler;
import java.util.logging.Level;
import java.util.logging.LogRecord;
import java.util.logging.Logger;
import java.util.logging.SimpleFormatter;

import java.util.ArrayList;
import java.util.Date;
import java.util.concurrent.Callable;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.LinkedBlockingQueue;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

// Why is support for logging in this package? Because the UVC
// camera code needs to run as a stand-alone app as well as in the FTC robotics
// environment.

//https://www.logicbig.com/tutorials/core-java-tutorial/logging/customizing-default-format.html
public class RobotLogCommon {

    private static final String TAG = "RobotCommonLog";
    private static final String FTCTAG = "FTCAuto";

    private static final Level DEFAULT_LEVEL = Level.FINE;
    private static Logger logger;
    private static FileHandler fileHandler;

    private static CompletableFuture<Void> logWriterFuture;
    private static final Lock logQueueLock = new ReentrantLock();

    private static final LinkedBlockingQueue<Pair<Level, String>> logEntryQueue = new LinkedBlockingQueue<>();
    private static boolean loggerInitialized = false;
    private static boolean closeLogWriter = false;

    // Wanted to use a MemoryHandler to log to a buffer but found out here --
    // https://chromium.googlesource.com/android_tools/+/refs/heads/master/sdk/sources/android-25/java/util/logging/MemoryHandler.java
    // that push() is synchronous with the linked FileHandler. So we'll use a
    // BlockingQueue instead.
    public static void initialize(String pLogPath) {
        if (loggerInitialized)
            return;

        try {
            logger = Logger.getLogger(RobotLogCommon.class.getName());

            // System.setProperty("java.util.logging.config.file",
            // "Files/logging.properties");
            // OR
            // FileInputStream logProperties = new
            // FileInputStream("Files/logging.properties");
            // LogManager.getLogManager().readConfiguration(logProperties);

            // 11/27/2019 When I use either of the above methods to set the log file size
            // limit, the file count, and the append flag - only the append flag is
            // honored. A single log file is created that grows indefinitely. But if I
            // use the next line, everything works.
            fileHandler = new FileHandler(pLogPath + "FTCRobotLog.txt", 1000000, 5, true);
            logger.setUseParentHandlers(false);
            fileHandler.setFormatter(new SimpleFormatter() {
                // original private static final String format = "[%1$tF %1$tT] [%2$-7s] %3$s
                // %n";
                // outputs [2019-08-29 13:46:11.496]
                private static final String format = "[%1$tF %1$tT.%1$tL] [%2$-7s] %3$s %n";

                // outputs August 29, 2019 1:37:10.810 PM
                // private static final String format = "[%1$tb %1$td, %1$tY
                // %1$tl:%1$tM:%1$tS.%1$tL %1$Tp] [%2$-7s] %3$s %n";

                @Override
                public synchronized String format(final LogRecord lr) {
                    return String.format(format, new Date(lr.getMillis()), lr.getLevel().getLocalizedName(),
                            lr.getMessage());
                }
            });

            logger.addHandler(fileHandler);
            logger.setLevel(DEFAULT_LEVEL);

            // Start a CompletableFuture for writing out the log.
            LogWriter logWriter = new LogWriter();
            logWriterFuture = CompletableFuture.supplyAsync(logWriter::call);

            loggerInitialized = true;

        } catch (Exception exception) {
            throw new AutonomousRobotException(TAG, "Error in initialization of logging: " + exception.getMessage());
        }
    }

    public static void setMinimimLoggingLevel(final Level pLoggingLevel) {
        logger.setLevel(pLoggingLevel);
    }

    public static Level getMinimumLoggingLevel() {
        return logger.getLevel();
    }

    // Test that the string parameter corresponds to a logging
    // Level that is >= the current minimum.
    public static boolean logThisLevel(final String pLevel) {
        Level levelToCheck;
        switch (pLevel) {
            case "d": {
                levelToCheck = Level.FINE;
                break;
            }
            case "v": {
                levelToCheck = Level.FINER;
                break;
            }
            case "vv": {
                levelToCheck = Level.FINEST;
                break;
            }
            default: {
                throw new AutonomousRobotException(TAG, "Invalid debug log option");
            }
            /*
            // IntelliJ Java 14 only
            case "d" -> levelToCheck = Level.FINE;
            case "v" -> levelToCheck = Level.FINER;
            case "vv" -> levelToCheck = Level.FINEST;
            default -> throw new AutonomousRobotException(TAG, "Invalid debug log option");
            */
        }

        return logger.isLoggable(levelToCheck);
    }

    public static void e(String pTAG, String pLogMessage) {
        enqueueLogEntry(Level.SEVERE, pTAG + " " + pLogMessage);
    }

    public static void c(String pTAG, String pLogMessage) {
        enqueueLogEntry(Level.CONFIG, pTAG + " " + pLogMessage);
    }

    public static void i(String pTAG, String pLogMessage) {
        enqueueLogEntry(Level.INFO, pTAG + " " + pLogMessage);
    }

    public static void d(String pTAG, String pLogMessage) {
        enqueueLogEntry(Level.FINE, pTAG + " " + pLogMessage);
    }

    public static void v(String pTAG, String pLogMessage) {
        enqueueLogEntry(Level.FINER, pTAG + " " + pLogMessage);
    }

    public static void vv(String pTAG, String pLogMessage) {
        enqueueLogEntry(Level.FINEST, pTAG + " " + pLogMessage);
    }

    // Even though the BlockingQueue is thread-safe there's a warning about drainTo:
    // "A failure encountered while attempting to add elements to collection c
    // [during
    // the drainTo] may result in elements being in neither, either or both
    // collections
    // when the associated exception is thrown."
    private static void enqueueLogEntry(Level pLevel, String pLogString) {
        if (!loggerInitialized) {
            System.out.println(pLogString); // desperation time
            return;
        }

        logQueueLock.lock();
        try {
            logEntryQueue.add(Pair.create(pLevel, pLogString));
        } finally {
            logQueueLock.unlock();
        }
    }

    public static void closeLog() {
        if (!loggerInitialized) {
            System.out.println(TAG + " Can't close a log that's not open"); // desperation time
            return;
        }

        // See comments above at enqueueLogEntry.
        logQueueLock.lock();
        try {
            if (closeLogWriter)
                return; // only close once
            closeLogWriter = true;
            logEntryQueue.add(Pair.create(Level.INFO, TAG + " Shutting down log"));
        } finally {
            logQueueLock.unlock();
        }

        try {
            // Wait for the log writer to shut down.
            // Use a timeout value so that we never get hung up here.
            logWriterFuture.get(250, TimeUnit.MILLISECONDS);
        } catch (Throwable t) {
            if (t instanceof TimeoutException) {
                logger.info(TAG + " Timed out waiting for the final log entries to be written out");
            }
            // Swallow all other exceptions - don't take down the application because of a logging problem.
        } finally {
            fileHandler.close();
        }
    }

    private static class LogWriter implements Callable<Void> {
        public Void call() {
            ArrayList<Pair<Level, String>> drain = new ArrayList<>();
            boolean closeNow = false;
            while (true) {
                try {
                    drain.clear();
                    drain.add(logEntryQueue.take()); // block here
                    logQueueLock.lock();
                    // Will get queue entries beyond the first.
                    logEntryQueue.drainTo(drain);

                    if (closeLogWriter) {
                        closeNow = true;
                        logger.info(TAG + " Closing log and writing out the last " + drain.size() + " entries on the queue");
                    }

                } catch (InterruptedException iex) {
                    // Swallow exception - we don't want to take down the application because of a
                    // logging problem.
                } finally {
                    logQueueLock.unlock();
                }

                //logger.info("Logging " + drain.size() + " drained queue entries");

                /*
                  Java SEVERE (highest value) -> Android Log.e
                  WARNING -> Log.w
                  INFO -> Log.i
                  CONFIG -> Log.i
                  FINE -> Log.d
                  FINER -> Log.v
                  FINEST (lowest value) -> Log.v
                  Android Log.x takes String tag, String message
                */
                drain.forEach(e -> {
                            logger.log(e.first, e.second);

                            // Output to Android logcat
                            // Map Java logging levels to Android
                            String javaLoggingLevel = e.first.toString();
                            switch (javaLoggingLevel) {
                                case "SEVERE": {
                                    Log.e(FTCTAG, e.second);
                                    break;
                                }
                                case "WARNING": {
                                    Log.w(FTCTAG, e.second);
                                    break;
                                }
                                case "INFO":
                                case "CONFIG": {
                                    Log.i(FTCTAG, e.second);
                                    break;
                                }
                                 case "FINE": {
                                    Log.d(FTCTAG, e.second);
                                    break;
                                }
                                case "FINER":
                                case "FINEST": {
                                    Log.v(FTCTAG, e.second);
                                    break;
                                }
                                default: {
                                    throw new AutonomousRobotException(TAG, "Invalid debug log option");
                                }
                            }
                        }
                );

                if (closeNow)
                    return null; // stop now
            }
        }
    }
}
