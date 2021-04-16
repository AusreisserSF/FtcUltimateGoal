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
import org.firstinspires.ftc.teamcode.math.Angle;
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
import java.util.concurrent.locks.Condition;
import java.util.concurrent.locks.ReentrantLock;
import java.util.stream.Collectors;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

// Class that continuously reads Vumark data and posts the results.
//## Not designed as a Singleton and not intended that an instance
//## of this class be shared between thread.
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
    public enum SupportedVumark {BLUE_TOWER_GOAL, RED_TOWER_GOAL, RED_ALLIANCE, BLUE_ALLIANCE, FRONT_WALL}

    // From ConceptVuforiaUltimateGoalNavigationWebcam.java
     /*
        VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
        blueTowerGoalTarget.setName("Blue Tower Goal Target");
        VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
        redTowerGoalTarget.setName("Red Tower Goal Target");
        VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
        redAllianceTarget.setName("Red Alliance Target");
        VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
        blueAllianceTarget.setName("Blue Alliance Target");
        VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
        frontWallTarget.setName("Front Wall Target");
     */

    // Trackables index 0
    public static final String BLUE_TOWER_GOAL = "BLUE_TOWER_GOAL"; // girls' basketball

    // Trackables index 1
    public static final String RED_TOWER_GOAL = "RED_TOWER_GOAL"; // volleyball

    // Trackables index 2
    public static final String RED_ALLIANCE = "RED_ALLIANCE"; // equipment

    // Trackables index 3
    public static final String BLUE_ALLIANCE = "BLUE_ALLIANCE"; // soccer celebration

    // Trackables index 4
    public static final String FRONT_WALL = "FRONT_WALL"; // hands

    // Use the values from the Vumark sample: they reflect the position of the camera on the front of the robot.
    private final int CAMERA_FORWARD_DISPLACEMENT = 185;   // eg: AutoCamera is 110 mm in front of robot center
    private final int CAMERA_VERTICAL_DISPLACEMENT = 200;   // eg: AutoCamera is 200 mm above ground
    private final int CAMERA_LEFT_DISPLACEMENT = -50;     // eg: AutoCamera is ON the robot's center line

    // For reading Vuforia Vumarks.
    private final LinearOpMode linearOpMode;
    private boolean vumarksActivated = false;

    // Thread-related.
    private CountDownLatch countDownLatch;
    private VumarkReaderCallable vumarkReaderCallable;
    private CompletableFuture<Void> vumarkReaderFuture;
    private final ReentrantLock vumarkLock = new ReentrantLock();
    private final Condition vumarkCondition = vumarkLock.newCondition();

    private final VuforiaTrackables targetsUltimateGoal;
    private final List<VuforiaTrackable> allTrackables = new ArrayList<>();

    // Store Vumark data separately so that it can be retrieved separately.
    public static final int VUMARK_DEQUE_DEPTH = 19;
    private final Map<SupportedVumark, Deque<Pose>> trackedVumarks = new HashMap<>();

    //**TODO caller sends in list of active Vumarks derived from XML.
    public VumarkReader(LinearOpMode pLinearOpMode, VuforiaLocalizer pAutoVuforiaLocalizer,
                        List<SupportedVumark> pVumarksOfInterest) {
        linearOpMode = pLinearOpMode;

        // From the FTC sample ConceptVuforiaUltimateGoalNavigationWebcam.java
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

        // Iterate through the List of active Vumark enumeration values for the
        // selected OpMode.
        targetsUltimateGoal = pAutoVuforiaLocalizer.loadTrackablesFromAsset("UltimateGoal");
        for (SupportedVumark vumark : pVumarksOfInterest) {
            // Allocate a Deque for each Vumark of interest.
            trackedVumarks.put(vumark, new ArrayDeque<>(VUMARK_DEQUE_DEPTH)); // empty queue
            switch (vumark) {
                case BLUE_TOWER_GOAL: {
                    VuforiaTrackable blueTowerGoalTarget = targetsUltimateGoal.get(0);
                    blueTowerGoalTarget.setName(BLUE_TOWER_GOAL);

                    // The tower goal targets are located a quarter field length from the ends of the back perimeter wall.
                    blueTowerGoalTarget.setLocation(OpenGLMatrix
                            .translation(halfField, quadField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

                    allTrackables.add(blueTowerGoalTarget);
                    break;
                }
                case RED_TOWER_GOAL: {
                    VuforiaTrackable redTowerGoalTarget = targetsUltimateGoal.get(1);
                    redTowerGoalTarget.setName(RED_TOWER_GOAL);

                    redTowerGoalTarget.setLocation(OpenGLMatrix
                            .translation(halfField, -quadField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

                    allTrackables.add(redTowerGoalTarget);
                    break;
                }
                case RED_ALLIANCE: {
                    VuforiaTrackable redAllianceTarget = targetsUltimateGoal.get(2);
                    redAllianceTarget.setName(RED_ALLIANCE);

                    redAllianceTarget.setLocation(OpenGLMatrix
                            .translation(0, -halfField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

                    allTrackables.add(redAllianceTarget);
                    break;
                }
                case BLUE_ALLIANCE: {
                    VuforiaTrackable blueAllianceTarget = targetsUltimateGoal.get(3);
                    blueAllianceTarget.setName(BLUE_ALLIANCE);

                    blueAllianceTarget.setLocation(OpenGLMatrix
                            .translation(0, halfField, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

                    allTrackables.add(blueAllianceTarget);
                    break;
                }
                case FRONT_WALL: {
                    VuforiaTrackable frontWallTarget = targetsUltimateGoal.get(4);
                    frontWallTarget.setName(FRONT_WALL);

                    frontWallTarget.setLocation(OpenGLMatrix
                            .translation(-halfField, 0, mmTargetHeight)
                            .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

                    allTrackables.add(frontWallTarget);
                    break;
                }
                default: {
                    throw new AutonomousRobotException(TAG, "Unrecognized Vumark " + vumark);
                }
            }
        }

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
    public void activateVumarkRecognition() throws InterruptedException {
        if (vumarksActivated)
            return; // nothing to do
        vumarksActivated = true;

        targetsUltimateGoal.activate();
        trackedVumarks.entrySet().forEach(e -> e.getValue().clear()); // clear all deques

        // Start up the Vumark reader as a CompletableFuture.
        RobotLogCommon.d(TAG, "Starting Vumark reader thread");

        countDownLatch = new CountDownLatch(1);
        vumarkReaderCallable = new VumarkReaderCallable();
        vumarkReaderFuture = CommonUtils.launchAsync(vumarkReaderCallable);

        // Wait here until the thread is off and running.
        countDownLatch.await();
        RobotLogCommon.d(TAG, "Wait for CountDownLatch done; Vumark reader thread is running");
    }

    // Turn off when done with Vumark recognition.
    public void deactivateVumarkRecognition() throws IOException, InterruptedException {
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

        RobotLogCommon.d(TAG, "Looking for Vumark " + pVumark + " with timeout value " + pTimeout + " ms");
        if (!trackedVumarks.containsKey(pVumark))
            throw new AutonomousRobotException(TAG, "Requested Vumark " + pVumark + " not not been selected as a Vumark of interest for the current OpMode");

        vumarkLock.lock();
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
                RobotLogCommon.d(TAG, "Timed out waiting for Vumark " + pVumark + " with timeout value " + pTimeout + " ms");
                return Optional.empty();
            }

            // Get the most recent Vumark of the requested type from its deque.
            return Optional.of(trackedVumarks.get(pVumark).getLast());
        } finally {
            vumarkLock.unlock();
            RobotLogCommon.d(TAG, "Exiting getMostRecentVumarkPose");
        }
    }

    // Creates a median Pose from the requested number of samples for the requested Vumark.
    // This method is intended to be used when the robot is stationary. If the timeout
    // expires before any samples have been collected, this method returns an empty Optional.
    // If at least one sample has been collected but the timeout expires before the requested
    // number has been collected, this method returns the median of the collected samples.
    public Optional<Pose> getMedianVumarkPose(SupportedVumark pVumark, int pTimeout, int pNumSamples) throws InterruptedException {

        RobotLogCommon.d(TAG, "Calculating median for Vumark " + pVumark + " over " + pNumSamples + " samples with a timeout value of " + pTimeout + " ms");
        if (!trackedVumarks.containsKey(pVumark))
            throw new AutonomousRobotException(TAG, "Requested Vumark " + pVumark + " not not been selected as a Vumark of interest for the current OpMode");

        vumarkLock.lock();
        if (!vumarksActivated)
            throw new AutonomousRobotException(TAG, "getMedianVumarkPose(): Vuforia is not activated");

        trackedVumarks.get(pVumark).clear(); // start fresh
        int entriesToCollect = pNumSamples <= 0 || pNumSamples > VUMARK_DEQUE_DEPTH ? VUMARK_DEQUE_DEPTH : pNumSamples;
        try {
            boolean waitVal = true;
            long now = System.currentTimeMillis();
            long deadline = now + pTimeout;
            while (trackedVumarks.get(pVumark).isEmpty() && (now < deadline)) {
                waitVal = vumarkCondition.await(deadline - now, TimeUnit.MILLISECONDS);
                if (!waitVal)
                    break; // timed out while waiting for the first Vumark
                now = System.currentTimeMillis();
            }

            // Did we collect any samples at all?
            if (!waitVal || (now >= deadline)) {
                RobotLogCommon.d(TAG, "Timed out waiting for Vumark " + pVumark + " with timeout value " + pTimeout + " ms");
                return Optional.empty(); // no
            }

            // The queue may contain one or more Vumarks. Drain them to a local collection.
            List<Pose> collectedVumarks = new ArrayList<>(trackedVumarks.get(pVumark));
            int samplesCollected = collectedVumarks.size();
            trackedVumarks.get(pVumark).clear(); // start fresh for additional Vumarks

            // Keep collecting Vumarks until we reach the requested number of samples or
            // we time out.
            while ((trackedVumarks.get(pVumark).size() + samplesCollected) < entriesToCollect && (now < deadline)) {
                waitVal = vumarkCondition.await(deadline - now, TimeUnit.MILLISECONDS);
                if (!waitVal)
                    break; // timed out while waiting for additional Vumarks
                now = System.currentTimeMillis();
            }

            // Reached the requested number of samples or timed out.
            // The queue may contain zero or more Vumarks. Drain them to a local collection.
            List<Pose> additionalVumarks = new ArrayList<>(trackedVumarks.get(pVumark));

            vumarkLock.unlock(); // release the lock and work only with local data

            // After the next line we have a collection of Pose instances. We want the median
            // of each component of the Pose: x, y, and rotation.
            collectedVumarks.addAll(additionalVumarks); // combine

            // But first trim the final collection to the requested number of Vumarks.
            int vumarksToSkip = collectedVumarks.size() - entriesToCollect;
            List<Pose> finalVumarkCollection = (vumarksToSkip <= 0) ? collectedVumarks :
                collectedVumarks.stream().skip(vumarksToSkip).collect(Collectors.toCollection(ArrayList::new));

            // Found two stackoverflow posts that helped:
            // https://stackoverflow.com/questions/43667989/finding-the-median-value-from-a-list-of-objects-using-java-8
            // and
            // https://stackoverflow.com/questions/10791568/calculating-average-of-an-array-list
            RobotLogCommon.d(TAG, "Calculating the median for data from " + finalVumarkCollection.size() + " Vumarks");
            double medianX = finalVumarkCollection
                    .stream().map(p -> p.x)
                    .mapToDouble(x -> x).sorted()
                    .skip((finalVumarkCollection.size() - 1) / 2).limit(2 - finalVumarkCollection.size() % 2).average().getAsDouble();

            double medianY = finalVumarkCollection
                    .stream().map(p -> p.y)
                    .mapToDouble(y -> y).sorted()
                    .skip((finalVumarkCollection.size() - 1) / 2).limit(2 - finalVumarkCollection.size() % 2).average().getAsDouble();

            double medianR = finalVumarkCollection
                    .stream().map(p -> p.r)
                    .mapToDouble(r -> r).sorted()
                    .skip((finalVumarkCollection.size() - 1) / 2).limit(2 - finalVumarkCollection.size() % 2).average().getAsDouble();

            return Optional.of(new Pose(medianX, medianY, medianR));
        } finally {
            if (vumarkLock.isHeldByCurrentThread())
                vumarkLock.unlock();
        }
    }

    // Assumes that the robot's current position and target position are
    // given in FTC field coordinates, which is a Cartesian system with
    // 0,0 at the center of the field, the x-axis parallel to the red
    // alliance wall and increasing to the right, and the y-axis
    // perpendicular to the red wall and increasing away from the red
    // wall. This method is suitable for use with the x and y values
    // returned from Vumarks.
    // Returns an angle adjusted so that it is relative to a heading of
    // zero at the top of a circle. The angle is otherwise not normalized
    // to the standard FTC magnitude and direction.
    // Note: the rotation value in the Pose is not used here.
    public RobotVector getDistanceAndAngleToTarget(Pose pRobotCurrentPosition, Pose pRobotTargetPosition) {

        RobotLogCommon.d(TAG, "Get distance and angle from x " + pRobotCurrentPosition.x + ", y " + pRobotCurrentPosition.y + " to x " +
                pRobotTargetPosition.x + ", y " + pRobotTargetPosition.y);

        // Get the Euclidean distance from the angle to the target.
        double deltaX = pRobotTargetPosition.x - pRobotCurrentPosition.x;
        double deltaY = pRobotTargetPosition.y - pRobotCurrentPosition.y;
        double distance = Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2));

        // Get the angle between the current position and the target position.
        // reference
        // https://stackoverflow.com/questions/2676719/calculating-the-angle-between-the-line-defined-by-two-points
        //delta_x = touch_x - center_x
        //delta_y = touch_y - center_y
        //theta_radians = atan2(delta_y, delta_x)

        // So in our case
        double thetaRadians = Math.atan2(deltaY, deltaX);
        double thetaDegrees = Math.toDegrees(thetaRadians);

        // Since in the reference "theta is measured counter-clockwise from the +x axis"
        // we need to subtract 90 degrees.
        thetaDegrees -= 90.0;

        RobotLogCommon.d(TAG,"Distance to target " + distance + ", angle " + thetaDegrees + " degrees");
        return new RobotVector(distance, thetaDegrees);
    }

    public static class RobotVector {
        public final double distanceToTarget;
        public final Angle angleToTarget;

        public RobotVector(double pDistanceToTarget, double degreesToTarget) {
            distanceToTarget = pDistanceToTarget;
            angleToTarget = new Angle(degreesToTarget, DEGREES);
        }
    }

    // Looks for a Vumark and, if present, makes the pose data available
    // to the main thread.
    private class VumarkReaderCallable extends AutoWorker<Void> {

        VumarkReaderCallable() {
            super();
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
            while (linearOpMode.opModeIsActive() && !stopThreadRequested()) {
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
                    // Comment from the example ConceptVuforiaNavigation.java.
                        /*
                          getUpdatedRobotLocation() will return null if no new information is available since
                          the last time that call was made, or if the trackable is not currently visible.
                          getRobotLocation() will return null if the trackable is not currently visible.
                         */
                    robotTransformationMatrix = ((VuforiaTrackableDefaultListener) trackable.getListener()).getRobotLocation();
                    vumarkLock.lock();
                    try {
                        if (robotTransformationMatrix == null) {
                            trackedVumarks.get(supportedVumark).clear();
                            RobotLogCommon.v(TAG, "Got empty location information from Vumark " + trackableName);
                            continue;
                        }

                        // Got a Vumark. Extract and repackage the relevant information.
                        RobotLogCommon.v(TAG, "Got location information from Vumark " + trackableName);
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
