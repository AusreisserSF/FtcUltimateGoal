package org.firstinspires.ftc.teamcode.auto.vision;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.CommonUtils;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.math.Pose;

import java.io.IOException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Deque;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.Collectors;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

// Class that continuously reads Vumark data and posts the results.
public class VumarkReader {

    private static final String TAG = "VumarkReader";

    // Reference code ConceptVuforiaUltimateGoalNavigation.java
    // in C:\FtcUltimateGoal\FtcRobotController\ftc-app\FtcRobotController\src\main\java\org\firstinspires\ftc\robotcontroller\external\samples

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float MM_PER_INCH = 25.4f;
    private static final float mmTargetHeight = (6) * MM_PER_INCH; // the height of the center of the target image above the floor

    // Constants for perimeter targets
    private static final float halfField = 72 * MM_PER_INCH;
    private static final float quadField = 36 * MM_PER_INCH;

    // Vumark identifiers
    public enum SupportedVumark {BLUE_TOWER_GOAL, FRONT_WALL}

    // The values below are used to determine whether the reported Vumark angle places the robot to the CW (left) or CCW side
    // (right) of dead center.

    // Trackables index 0
    public static final String BLUE_TOWER_GOAL = "BLUE_TOWER_GOAL";
    public static final double BLUE_TOWER_CW = 0.0;
    public static final double BLUE_TOWER_DEAD_CENTER = 90.0;
    public static final double BLUE_TOWER_CCW = 180;

    //## Because we're only running the blue alliance during the virtual
    // competitions, comment out the red Vumarks.
    // Trackables index 1
    //public static final String RED_TOWER_GOAL = "RED_TOWER_GOAL";
    //public static final double RED_TOWER_CW = 0.0;
    // public static final double RED_TOWER_DEAD_CENTER = 90.0;
    // public static final double RED_TOWER_CCW = 180.0;

    // Trackables index 4
    public static final String FRONT_WALL = "FRONT_WALL";
    public static final double FRONT_WALL_CW = 180.0;
    public static final double FRONT_WALL_DEAD_CENTER = 90.0;
    public static final double FRONT_WALL_CCW = 0.0;

    // Use the values from the Vumark sample: they reflect the position of the camera on the front of the robot.
    private final int CAMERA_FORWARD_DISPLACEMENT = 110;   // eg: AutoCamera is 110 mm in front of robot center
    private final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: AutoCamera is 200 mm above ground
    private final int CAMERA_LEFT_DISPLACEMENT = 0;     // eg: AutoCamera is ON the robot's center line

    // For reading Vuforia Vumarks.
    private final LinearOpMode linearOpMode;
    private boolean vumarksActivated = false;

    // Thread-related.
    private final CountDownLatch countDownLatch = new CountDownLatch(1);
    private VumarkReaderCallable vumarkReaderCallable;
    private CompletableFuture<Void> vumarkReaderFuture;
    private final ReentrantLock vumarkLock = new ReentrantLock();
    private final Condition vumarkCondition = vumarkLock.newCondition();
    private final AtomicBoolean stopVumarkReader = new AtomicBoolean();

    private final VuforiaTrackables targetsUltimateGoal;
    private final List<VuforiaTrackable> allTrackables = new ArrayList<>();

    // Store Vumark data separately so that it can be retrieved separately.
    public static final int VUMARK_DEQUE_DEPTH = 19;
    private final Map<SupportedVumark, Deque<Pose>> trackedVumarks = new HashMap<>();

    public VumarkReader(LinearOpMode pLinearOpMode, VuforiaLocalizer pAutoVuforiaLocalizer,
                        List<SupportedVumark> pVumarksToTrack) {
        linearOpMode = pLinearOpMode;
        pVumarksToTrack.forEach(v -> trackedVumarks.put(v, new ArrayDeque<Pose>(VUMARK_DEQUE_DEPTH)));

        targetsUltimateGoal = pAutoVuforiaLocalizer.loadTrackablesFromAsset("UltimateGoal");
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName(BLUE_TOWER_GOAL);

        //VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        //redTowerGoalTarget.setName(RED_TOWER_GOAL);

        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("FrontWallTarget");

        // To add all allTrackables.addAll(targetsUltimateGoal);
        allTrackables.add(blueTowerGoalTarget);
        //## red allTrackables.add(redTowerGoalTarget);
        allTrackables.add(frontWallTarget);

        /*
          In order for localization to work, we need to tell the system where each target is on the field, and
          where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
          Transformation matrices are a central, important concept in the math here involved in localization.
          See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
          for detailed information. Commonly, you'll encounter transformation matrices as instances
          of the {@link OpenGLMatrix} class.

          If you are standing in the Red Alliance Station looking towards the center of the field,
              - The X axis runs from your left to the right. (positive from the center to the right)
              - The Y axis runs from the Red Alliance Station towards the other side of the field
                where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
              - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)

          Before being transformed, each target image is conceptually located at the origin of the field's
           coordinate system (the center of the field), facing up.
         */

        // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
        blueTowerGoalTarget.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        frontWallTarget.setLocation(OpenGLMatrix
                .translation(-halfField, 0, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        //## redTowerGoalTarget.setLocation(OpenGLMatrix
        //     .translation(halfField, -quadField, mmTargetHeight)
        //     .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        /*
          Create a transformation matrix describing where the phone is on the robot.

          The coordinate frame for the robot looks the same as the field.
          The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
          Z is UP on the robot.  This equates to a bearing angle of Zero degrees.

          The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
          pointing to the LEFT side of the Robot.  It's very important when you test this code that the top of the
          camera is pointing to the left side of the  robot.  The rotation angles don't work if you flip the phone.

          If using the rear (High Res) camera:
          We need to rotate the camera around it's long axis to bring the rear camera forward.
          This requires a negative 90 degree rotation on the Y axis

          If using the Front (Low Res) camera
          We need to rotate the camera around it's long axis to bring the FRONT camera forward.
          This requires a Positive 90 degree rotation on the Y axis

          Next, translate the camera lens to where it is on the robot.
          In this example, it is centered (left to right), but 110 mm forward of the middle of the robot, and 200 mm above ground level.
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, -90, 0, 0));

        // Let all the trackable listeners know where the phone is.
        for (VuforiaTrackable trackable : allTrackables) {
            ((VuforiaTrackableDefaultListener) trackable.getListener()).setPhoneInformation(phoneLocationOnRobot, BACK);
        }
    }

    // Turn on Vumark recognition.
    public synchronized void activateVumarkRecognition() throws InterruptedException {
        if (vumarksActivated)
            return; // nothing to do
        vumarksActivated = true;

        targetsUltimateGoal.activate();
        trackedVumarks.entrySet().forEach(e -> e.getValue().clear()); // clear all deques

        // Start up the Vumark reader as a CompletableFuture.
        RobotLogCommon.d(TAG, "Starting Vumark reader thread");

        vumarkReaderCallable = new VumarkReaderCallable(countDownLatch);
        vumarkReaderFuture = CommonUtils.launchAsync(vumarkReaderCallable);

        // Wait here until the thread is off and running.
        countDownLatch.await();
    }

    // Turn off when done with Vumark recognition.
    public synchronized void deactivateVumarkRecognition() throws IOException, InterruptedException {
        if (!vumarksActivated)
            return; // nothing to do
        vumarksActivated = false;

        // Stop the reader and wait for it to complete.
        vumarkReaderCallable.stopThread();
        targetsUltimateGoal.deactivate();
        CommonUtils.getFutureCompletion(vumarkReaderFuture);
        trackedVumarks.entrySet().forEach(e -> e.getValue().clear()); // clear all deques
    }

    // Returns the robot's pose (x, y, and rotation) from the most recently read Vumark or an
    // empty Optional if no Vumark is in view and the timer expires.
    public Optional<Pose> getMostRecentVumarkPose(SupportedVumark pVumark, int pTimeout) throws InterruptedException {

        vumarkLock.lock();

        RobotLogCommon.d(TAG, "Looking for Vumark " + pVumark + " with timeout value " + pTimeout + " ms");
        if (!vumarksActivated)
            throw new AutonomousRobotException(TAG, "getRobotLocationFromVumark(): Vuforia is not activated");

        try {
            boolean waitVal = true;
            long now = System.currentTimeMillis();
            long deadline = now + pTimeout;
            while (trackedVumarks.get(pVumark).isEmpty() && (now < deadline)) {
                waitVal = vumarkCondition.await(deadline - now, TimeUnit.MILLISECONDS);
                if (!waitVal)
                    break; // timed out
                now = System.currentTimeMillis();
            }

            if (!waitVal || (now >= deadline)) {
                // RobotLogCommon.d(TAG, "Timed out waiting for a Vumark " + pTimeout + " ms");
                return Optional.empty();
            }

            // Get the most recent Vumark of the requested type from its deque.
            return Optional.of(trackedVumarks.get(pVumark).getLast());
        } finally {
            vumarkLock.unlock();
            // RobotLogCommon.d(TAG, "Exiting getRobotLocationFromVumark");
        }
    }

    // Creates a median Pose from the most recent pNumSamples queue entries for the requested Vumark.
    public Optional<Pose> getMedianVumarkPose(SupportedVumark pVumark, int pTimeout, int pNumSamples) throws InterruptedException {

        vumarkLock.lock();

        RobotLogCommon.d(TAG, "Calculating median for Vumark " + pVumark + " with timeout value " + pTimeout + " ms over " + pNumSamples + " samples");
        if (!vumarksActivated)
            throw new AutonomousRobotException(TAG, "getRobotLocationFromVumark(): Vuforia is not activated");

        try {
            boolean waitVal = true;
            long now = System.currentTimeMillis();
            long deadline = now + pTimeout;
            while (trackedVumarks.get(pVumark).isEmpty() && (now < deadline)) {
                waitVal = vumarkCondition.await(deadline - now, TimeUnit.MILLISECONDS);
                if (!waitVal)
                    break; // timed out
                now = System.currentTimeMillis();
            }

            if (!waitVal || (now >= deadline)) {
                // RobotLogCommon.d(TAG, "Timed out waiting for a Vumark " + pTimeout + " ms");
                return Optional.empty();
            }

            //**TODO Actually you should collect Vumarks from Vuforia until you have
            // pNumSamples on the queue ...
            // Create the median Pose.
            // Collect the last pNumSamples entries from the queue.
            Deque<Pose> vumarkDeque = trackedVumarks.get(pVumark);
            int dequeSize = vumarkDeque.size();
            int entriesToCollect = pNumSamples > dequeSize || pNumSamples > VUMARK_DEQUE_DEPTH ? dequeSize : pNumSamples;
            List<Pose> medianCollection = vumarkDeque.stream().skip(Math.max(0, dequeSize - entriesToCollect)).collect(Collectors.toList());

            vumarkLock.unlock();

            // Now we have a collection of Pose instances. We want the median of each component of the Pose: x, y, and rotation.
            // Found two stackoverflow posts that helped:
            // https://stackoverflow.com/questions/43667989/finding-the-median-value-from-a-list-of-objects-using-java-8
            // and
            // https://stackoverflow.com/questions/10791568/calculating-average-of-an-array-list
            double medianX = medianCollection
                    .stream().map(p -> p.x)
                    .mapToDouble(x -> x).sorted()
                    .skip((medianCollection.size() - 1) / 2).limit(2 - medianCollection.size() % 2).average().getAsDouble();

            double medianY = medianCollection
                    .stream().map(p -> p.y)
                    .mapToDouble(y -> y).sorted()
                    .skip((medianCollection.size() - 1) / 2).limit(2 - medianCollection.size() % 2).average().getAsDouble();

            double medianR = medianCollection
                    .stream().map(p -> p.r)
                    .mapToDouble(r -> r).sorted()
                    .skip((medianCollection.size() - 1) / 2).limit(2 - medianCollection.size() % 2).average().getAsDouble();

            return Optional.of(new Pose(medianX, medianY, medianR));
        } finally {
            if (vumarkLock.isHeldByCurrentThread())
                vumarkLock.unlock();
        }
    }

    // Looks for a Vumark and, if present, makes the pose data available
    // to the main thread.
    private class VumarkReaderCallable extends AutoWorker<Void> {

        VumarkReaderCallable(CountDownLatch pCountDownLatch) {
            super(pCountDownLatch);
        }

        public Void call() {
            RobotLogCommon.d(TAG, "In Vumark thread");
            countDownLatch.countDown(); // signal that I'm running

            // Read Vumarks continually. If a Vumark is not visible or a request
            // for its location returns null, clear its deque. Otherwise transform
            // its data into a Pose and put the Pose on the Vumark's deque.
            String trackableName;
            SupportedVumark supportedVumark;
            OpenGLMatrix robotTransformationMatrix;
            while (linearOpMode.opModeIsActive() && !stopVumarkReader.get()) {
                for (VuforiaTrackable trackable : allTrackables) {
                    trackableName = trackable.getName();
                    supportedVumark = SupportedVumark.valueOf(trackableName);
                    if (!((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible()) {
                        vumarkLock.lock();
                        trackedVumarks.get(supportedVumark).clear();
                        vumarkLock.unlock();
                        continue;
                    }

                    // A Vumark is visible. See if you can get the robot's location from it.
                    // This floods the log; don't use!! RobotLogCommon.d(TAG, "Vuforia trackable is visible " + trackableName);

                    // Comment from the example ConceptVuforiaNavigation.java.
                        /*
                          getUpdatedRobotLocation() will return null if no new information is available since
                          the last time that call was made, or if the trackable is not currently visible.
                          getRobotLocation() will return null if the trackable is not currently visible.
                         */
                    robotTransformationMatrix = ((VuforiaTrackableDefaultListener) trackable.getListener()).getRobotLocation();

                    // RobotLogCommon.v(TAG, "Got location information from Vumark " + trackableName);
                    vumarkLock.lock();
                    try {
                        if (robotTransformationMatrix == null) {
                            trackedVumarks.get(supportedVumark).clear();
                            continue;
                        }

                        // Got a Vumark. Extract and repackage the relevant information.
                        VectorF translation = robotTransformationMatrix.getTranslation();
                        double poseX = translation.get(0) / MM_PER_INCH;
                        double poseY = translation.get(1) / MM_PER_INCH;
                        // double poseZ = translation.get(2) / MM_PER_INCH;

                        // Get the rotation of the robot in degrees.
                        Orientation rotation = Orientation.getOrientation(robotTransformationMatrix, EXTRINSIC, XYZ, DEGREES);
                        double poseR = rotation.thirdAngle;

                        // Put the pose onto the correct deque. But if the deque is full throw away the
                        // oldest entry.
                        Pose pose = new Pose(poseX, poseY, poseR);
                        Deque<Pose> vumarkDeque = trackedVumarks.get(supportedVumark);
                        if (!vumarkDeque.offerLast(pose)) {
                            // The deque is full.
                            vumarkDeque.removeFirst(); // make room
                            vumarkDeque.offerLast(pose);
                        }

                        vumarkCondition.signal(); // let the main thread know
                    } finally {
                        vumarkLock.unlock();
                    }
                }
            }

            return null;
        }
    }

}
