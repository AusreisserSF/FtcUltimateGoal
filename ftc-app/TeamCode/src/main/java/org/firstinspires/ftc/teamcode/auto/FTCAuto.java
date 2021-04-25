package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.CommonUtils;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.RobotXMLElement;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.firstinspires.ftc.ftcdevcommon.android.WorkingDirectory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.auto.vision.FileImage;
import org.firstinspires.ftc.teamcode.auto.vision.ImageProvider;
import org.firstinspires.ftc.teamcode.auto.vision.RingParameters;
import org.firstinspires.ftc.teamcode.auto.vision.RingRecognition;
import org.firstinspires.ftc.teamcode.auto.vision.RingReturn;
import org.firstinspires.ftc.teamcode.auto.vision.VuforiaImage;
import org.firstinspires.ftc.teamcode.auto.vision.VuforiaWebcam;
import org.firstinspires.ftc.teamcode.auto.vision.VumarkReader;
import org.firstinspires.ftc.teamcode.auto.xml.AutoCommandXML;
import org.firstinspires.ftc.teamcode.auto.xml.RingParametersXML;
import org.firstinspires.ftc.teamcode.auto.xml.RobotActionXML;
import org.firstinspires.ftc.teamcode.auto.xml.TargetZoneXML;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.LCHSMath;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.LCHSRobot;
import org.firstinspires.ftc.teamcode.robot.OldRingShooter;
import org.firstinspires.ftc.teamcode.robot.WobbleArm;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import java.util.logging.Level;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathException;

import static android.os.SystemClock.sleep;

public class FTCAuto {

    private static final String TAG = "FTCAuto";

    private final LinearOpMode linearOpMode;
    private final LCHSRobot robot;

    private final RobotConstants.Alliance alliance;
    private final RobotConstantsUltimateGoal.OpMode autoOpMode;
    private final String workingDirectory;

    private final RobotActionXML actionXML;
    private final RobotActionXML.RobotActionData actionData; // for the selected OpMode

    // Webcam and Vumarks.
    private VuforiaWebcam vuforiaWebcam;
    private VuforiaLocalizer vuforiaLocalizer;
    private VumarkReader vumarkReader;

    // Ring recognition.
    private final RingParametersXML ringParametersXML;
    private final RingParameters ringParameters;
    private final RingRecognition ringRecognition;

    private final TargetZoneXML targetZoneXML;
    private final HashMap<RobotConstantsUltimateGoal.TargetZone, List<RobotXMLElement>> targetZoneCommands;
    private RobotConstantsUltimateGoal.TargetZone targetZone;
    private List<RobotXMLElement> targetZoneInsert = new ArrayList<>();
    private boolean executeTargetZoneActions = false;

    // Load OpenCV.
    private static boolean openCVInitialized = false;

    static {
        // Android only
        if (OpenCVLoader.initDebug())
            openCVInitialized = true;
    }

    // Main class for the autonomous run.
    public FTCAuto(RobotConstantsUltimateGoal.OpMode autoOpMode, RobotConstants.Alliance alliance, LinearOpMode pLinearOpMode)
            throws ParserConfigurationException, SAXException, XPathException, IOException, InterruptedException {

        RobotLogCommon.c(TAG, "FTCAuto constructor");

        // A failure in OpenCV initialization will prevent us from recognizing
        // the ring stack or the Vumark below a tower goal.
        if (!openCVInitialized)
            throw new AutonomousRobotException(TAG, "Error in OpenCV initialization");

        this.autoOpMode = autoOpMode;
        this.alliance = alliance;
        if (alliance == RobotConstants.Alliance.UNKNOWN)
            throw new AutonomousRobotException(TAG, "Alliance is UNKNOWN");

        workingDirectory = WorkingDirectory.getWorkingDirectory();

        // Initialize the hardware.
        linearOpMode = pLinearOpMode;
        robot = new LCHSRobot(linearOpMode);
        robot.initializeIMU();

        // Get the directory for the various configuration files.
        String xmlDirectory = workingDirectory + RobotConstants.xmlDir;

        // Read the robot action file for all opmodes.
        actionXML = new RobotActionXML(xmlDirectory);

        // Extract data from the parsed XML file for the selected OpMode only
        actionData = actionXML.getOpModeData(autoOpMode.toString());

        Level lowestLoggingLevel = actionData.lowestLoggingLevel;
        if (lowestLoggingLevel != null) // null means use the default
            RobotLogCommon.setMinimimLoggingLevel(lowestLoggingLevel);
        RobotLogCommon.c(TAG, "Lowest logging level " + RobotLogCommon.getMinimumLoggingLevel());

        // Start the asynchronous initialization of Vuforia.
        if (robot.webcam1Name != null) {
            RobotLogCommon.d(TAG, "Vuforia: start asynchronous initialization");
            vuforiaWebcam = new VuforiaWebcam(robot.webcam1Name);
        }

        // Read the ring stack image recognition parameters from an xml file.
        ringParametersXML = new RingParametersXML(xmlDirectory);
        ringParameters = ringParametersXML.getRingParameters();
        ringRecognition = new RingRecognition();

        // Read the XML file with the target zones for all OpModes.
        // For the current OpMode get the commands for all three target zones.
        // We won't know which zone to go for until we've performed image recognition.
        targetZoneXML = new TargetZoneXML(xmlDirectory);
        targetZoneCommands = targetZoneXML.getTargetZoneCommands(this.autoOpMode);

        // Wait for the asynchronous initialization of Vuforia to complete.
        if (robot.webcam1Name != null) {
            RobotLogCommon.d(TAG, "wait for initialization");
            vuforiaLocalizer = vuforiaWebcam.waitForVuforiaInitialization();

            // Prepare to read Vumarks but don't start yet.
            // Get the Vumarks of interest for the selected OpMode from the RobotAction.xml file.
            vumarkReader = new VumarkReader(linearOpMode, vuforiaLocalizer, actionData.vumarksOfInterest);
        }

        RobotLogCommon.c(TAG, "FTCAuto construction complete");
    }

    public void runRobot() throws XPathException, InterruptedException, IOException {

        RobotLogCommon.i(TAG, "At start");
        RobotLogCommon.i(TAG, "OpMode: " + autoOpMode + ", Alliance: " + alliance);

        // Safety check against ftc runtime initialization errors.
        // Make sure the opmode is still active.
        if (!linearOpMode.opModeIsActive())
            throw new AutonomousRobotException(TAG, "OpMode unexpectedly inactive in runRobot()");

        // Follow the choreography specified in the robot action file.
        List<RobotXMLElement> actions = actionData.actions;
        try {
            for (RobotXMLElement action : actions) {

                if (!executeTargetZoneActions) // execute Target Zone specific steps now?
                    doCommand(action); // no, but doCommand may change that

                // Takes care of the case where the EXECUTE_TARGET_ZONE_STEPS
                // action is the last command for the opmode in RobotConfig.xml.
                if (executeTargetZoneActions) { // any Target Zone specific steps?
                    // Yes, do all of those commands now.
                    for (RobotXMLElement insertedStep : targetZoneInsert) {
                        if (insertedStep.getRobotXMLElementName().equals("EXECUTE_TARGET_ZONE_STEPS"))
                            throw new AutonomousRobotException(TAG, "Nesting of EXECUTE_TARGET_ZONE_STEPS is not allowed");
                        doCommand(insertedStep);
                    }
                    targetZoneInsert.clear();
                    executeTargetZoneActions = false;
                }
            }
        } finally {
            if (vuforiaLocalizer != null) {
                RobotLogCommon.i(TAG, "Shutting down Vuforia");
                Objects.requireNonNull(vuforiaLocalizer.getCamera()).close();
            }

            if (vumarkReader != null)
                vumarkReader.deactivateVumarkRecognition();

            RobotLogCommon.i(TAG, "Exiting FTCAuto");
            linearOpMode.telemetry.addData("FTCAuto", "COMPLETE");
            linearOpMode.telemetry.update();
        }
    }

    //===============================================================================================
    //===============================================================================================

    // Using the XML elements and attributes from the configuration file RobotAction.xml,
    // execute the action.
    private void doCommand(RobotXMLElement pAction) throws InterruptedException, XPathException, IOException {

        // Set up XPath access to the current action command.
        XPathAccess commandXPath = new XPathAccess(pAction);
        String commandName = pAction.getRobotXMLElementName().toUpperCase();
        RobotLogCommon.d(TAG, "Executing FTCAuto command " + commandName);

        switch (commandName) {

            //**TODO Refactoring required - code duplicated in RobotActionCommon
            case "MOVE": {
                double targetClicks = commandXPath.getDouble("distance") * DriveTrain.CLICKS_PER_INCH;
                double marginClicks = commandXPath.getDouble("margin") * DriveTrain.CLICKS_PER_INCH; // stop when within {margin} clicks
                double power = commandXPath.getDouble("power");
                double minPower = commandXPath.getDouble("minpower", 0.2);
                Angle direction = AutoCommandXML.getAngle(commandXPath, "direction"); // direction angle; right is 0, up 90, left 180
                Angle targetHeading = AutoCommandXML.getAngle(commandXPath, "heading"); // robot's target heading angle while moving
                PIDController rPIDController = AutoCommandXML.getPIDController(commandXPath, targetHeading.getDegrees());

                // optional parameters
                double intakePower = commandXPath.getDouble("intakePower", 0);
                double rampPercent = commandXPath.getDouble("rampPercent", 1);

                robot.shooter.intakeMotor.setPower(intakePower);

                Pose drivePose = new Pose();
                drivePose.x = Math.sin(direction.getRadians());
                drivePose.y = Math.cos(direction.getRadians());

                robot.driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                double rampStartDistance = rampPercent * targetClicks;
                int currentClicks = Math.abs(robot.driveTrain.rb.getCurrentPosition());
                double rampPower = 1;

                while (Math.abs(currentClicks - targetClicks) > marginClicks) {

                    if (rampStartDistance - currentClicks < 0) {
                        rampPower = LCHSMath.clipPower(1 - (currentClicks - rampStartDistance) / (targetClicks - rampStartDistance), minPower);
                    }

                    Angle actualHeading = robot.imu.getHeading();
                    currentClicks = Math.abs(robot.driveTrain.rb.getCurrentPosition());
                    drivePose.r = -rPIDController.getCorrectedOutput(actualHeading.getDegrees());
                    robot.driveTrain.drive(drivePose, power * rampPower);
                }

                robot.driveTrain.stop();
                robot.shooter.intakeMotor.setPower(0);

                break;
            }
            // A normalized turn, i.e. a turn from 0 to +-180 degrees, which will always be the
            // shortest distance from the current heading to the desired heading.
            //**TODO Refactoring required - code duplicated in RobotActionCommon
            case "TURN": {
                Angle angle = AutoCommandXML.getAngle(commandXPath, "angle");
                double maxPower = commandXPath.getDouble("maxpower");
                double minPower = commandXPath.getDouble("minpower");
                double margin = commandXPath.getDouble("margin");
                double intakePower = commandXPath.getDouble("intakePower", 0);
                PIDController pidController = AutoCommandXML.getPIDController(commandXPath, angle.getDegrees());

                double currentDegrees = robot.imu.getIntegratedHeading().getDegrees();

                RobotLogCommon.d(TAG, "Turn by " + angle + " degrees");

                robot.shooter.intakeMotor.setPower(intakePower);

                double error = currentDegrees - angle.getDegrees();
                while (Math.abs(error) > margin) {
                    currentDegrees = robot.imu.getIntegratedHeading().getDegrees();
                    double pidCorrectedPower = -pidController.getCorrectedOutput(currentDegrees);
                    double power = LCHSMath.clipPower(pidCorrectedPower, minPower) * maxPower;
                    robot.driveTrain.drive(new Pose(0, 0, power));
                    error = currentDegrees - angle.getDegrees();
                }
                robot.driveTrain.stop();
                robot.shooter.intakeMotor.setPower(0);

                break;
            }

            //**TODO Refactoring required - code duplicated in RobotActionCommon
            case "WOBBLE_MOTOR": {
                String stringState = commandXPath.getString("state");
                WobbleArm.FlipState state = WobbleArm.FlipState.DROP;

                switch (stringState) {
                    case "out":
                        state = WobbleArm.FlipState.OUT;
                        break;
                    case "floating":
                        state = WobbleArm.FlipState.FLOATING;
                        break;
                    case "rest":
                        state = WobbleArm.FlipState.REST;
                        break;
                    case "in":
                        state = WobbleArm.FlipState.IN;
                        break;
                    case "intake":
                        state = WobbleArm.FlipState.INTAKE;
                        break;
                    case "drop":
                        state = WobbleArm.FlipState.DROP;
                        break;
                }
                robot.wobbleArm.setFlipState(state);
                break;
            }

            //**TODO Refactoring required - code duplicated in RobotActionCommon
            case "WOBBLE_SERVO": {
                String state = commandXPath.getString("state");
                robot.wobbleArm.servo.setState(state);
                break;
            }

            //**TODO Refactoring required - code duplicated in RobotActionCommon
            case "SHOOT": {
                double shootVelocity = commandXPath.getDouble("shootVelocity");
                double intakePower = commandXPath.getDouble("intakeVelocity");
                int waitTime = commandXPath.getInt("waitTime");
                int dip = commandXPath.getInt("shootVelocityDip");

                //optional parameters
                int maxShotCount = commandXPath.getInt("maxShotCount", 3);
                boolean powerShot = commandXPath.getBoolean("powerShot", false);
                int shotWaitTime = commandXPath.getInt("shotWaitTime", 500);
                int beginningWaitTime = commandXPath.getInt("beginningWaitTime", 500);

                boolean shotRing = false;


                robot.shooter.shootMotor.setVelocity(shootVelocity);
                robot.shooter.intakeMotor.setPower(intakePower);
                double currentVelocity = robot.shooter.shootMotor.getVelocity();
                while (currentVelocity < shootVelocity) {
                    currentVelocity = robot.shooter.shootMotor.getVelocity();
                    sleep(20);
                }

                robot.shooter.intakeMotor.setPower(0);

                sleep(beginningWaitTime);

                int shotCount = 0;
                linearOpMode.telemetry.addData("Shots ", shotCount);
                linearOpMode.telemetry.update();
                long timeout = System.currentTimeMillis() + waitTime;
                while (System.currentTimeMillis() < timeout) {
                    shotRing = false;
                    currentVelocity = robot.shooter.shootMotor.getVelocity();

                    robot.shooter.triggerServo.setState("out");

                    for (int i = 0; i < 50; i++) {
                        linearOpMode.telemetry.addData("Velocity", robot.shooter.shootMotor.getVelocity());
                        linearOpMode.telemetry.update();

                        if (robot.shooter.shootMotor.getVelocity() < dip) {
                            shotCount++;
                            linearOpMode.telemetry.addData("Shots ", shotCount);
                            linearOpMode.telemetry.addData("Velocity", robot.shooter.shootMotor.getVelocity());
                            linearOpMode.telemetry.update();

                            shotRing = true;

                            break;
                        }
                        sleep(1);

                    }

                    robot.shooter.triggerServo.setState("rest");

                    /*
                    if (!shotRing){

                        robot.shooter.moveElevatorDown();
                        sleep(500);
                        robot.shooter.moveElevatorUp();

                    }
                    
                     */

                    if (shotCount >= maxShotCount) {
                        linearOpMode.telemetry.addData("Done ", shotCount);
                        linearOpMode.telemetry.update();
                        break;
                    }
                    while (robot.shooter.shootMotor.getVelocity() < shootVelocity) {
                        currentVelocity = robot.shooter.shootMotor.getVelocity();
                        sleep(20);
                    }

                    sleep(shotWaitTime);
                }


                robot.shooter.shootMotor.setVelocity(0);
                break;
            }

            case "INTAKE": {
                double intakePower = commandXPath.getDouble("intakePower");
                int intakeTime = commandXPath.getInt("intakeTime");

                boolean intakeBackground = commandXPath.getBoolean("intakeBackground", false);
                double dipVelocity = commandXPath.getDouble("dip", 100);
                double outtakePower = commandXPath.getDouble("outtakePower", 0.5);

                if (intakeBackground){
                    robot.shooter.intakeMotor.setPower(intakePower);
                }
                else if (!intakeBackground) {
                    robot.shooter.intakeMotor.setPower(intakePower);

                    double timeWhenWeAreDone = System.currentTimeMillis() + intakeTime;

                    while (System.currentTimeMillis() < timeWhenWeAreDone) {
//
                    }

                    robot.shooter.intakeMotor.setPower(0);

                }

                break;

            }

            //**TODO Refactoring required - code duplicated in RobotActionCommon
            case "ELEVATOR_UP": {
                robot.shooter.moveElevatorUp();

                break;
            }

            //**TODO Refactoring required - code duplicated in RobotActionCommon
            case "ELEVATOR_DOWN": {
                robot.shooter.moveElevatorDown();

                break;
            }

            // For testing, take a picture and write it out to a file.
            case "TAKE_PICTURE": {
                if (vuforiaLocalizer == null)
                    throw new AutonomousRobotException(TAG, "Vuforia not initialized");

                ImageProvider imageProvider = new VuforiaImage(vuforiaLocalizer);
                Pair<Mat, Date> ringImage = imageProvider.getImage();
                if (ringImage.first == null) {
                    RobotLogCommon.d(TAG, "Unable to get image from the camera");
                    linearOpMode.telemetry.addData("Take picture:", "unable to get image from the camera");
                    linearOpMode.telemetry.update();
                    return;
                }

                RobotLogCommon.d(TAG, "Took a picture");
                String fileDate = CommonUtils.getDateTimeStamp(ringImage.second);
                String outputFilenamePreamble = workingDirectory + RobotConstants.imageDir + "Image_" + fileDate;

                // The image may be RGB (from a camera) or BGR ( OpenCV imread from a file).
                Mat imgOriginal = ringImage.first.clone();

                // If you don't convert RGB to BGR here then the _IMG.png file will be written
                // out with incorrect colors (gold will show up as blue).
                if (imageProvider.getImageFormat() == ImageProvider.ImageFormat.RGB)
                    Imgproc.cvtColor(imgOriginal, imgOriginal, Imgproc.COLOR_RGB2BGR);

                String imageFilename = outputFilenamePreamble + "_IMG.png";
                RobotLogCommon.d(TAG, "Writing image " + imageFilename);
                Imgcodecs.imwrite(imageFilename, imgOriginal);

                RobotLogCommon.d(TAG, "Image width " + imgOriginal.cols() + ", height " + imgOriginal.rows());
                linearOpMode.telemetry.addData("Take picture:", "successful");
                linearOpMode.telemetry.update();
                break;
            }

            // Use OpenCV to find the stack of rings and determine the Target Zone.
            case "RECOGNIZE_RINGS": {
                String imageProviderId = commandXPath.getStringInRange("ocv_image_provider", commandXPath.validRange("vuforia", "file"));
                RobotLogCommon.d(TAG, "Image provider " + imageProviderId);

                ImageProvider imageProvider;
                if (imageProviderId.equals("vuforia")) {
                    if (vuforiaLocalizer == null)
                        throw new AutonomousRobotException(TAG, "Vuforia not initialized");
                    imageProvider = new VuforiaImage(vuforiaLocalizer);
                } else {
                    // For testing: get the image from a file.
                    imageProvider = new FileImage(workingDirectory + RobotConstants.imageDir + ringParameters.imageParameters.file_name);
                }

                // Call the OpenCV subsystem.
                RingReturn ringReturn = ringRecognition.findGoldRings(imageProvider, ringParameters);
                targetZone = ringReturn.targetZone;

                RobotLogCommon.d(TAG, "Found Target Zone " + targetZone);
                linearOpMode.telemetry.addData("Found ", targetZone);
                linearOpMode.telemetry.update();

                // Prepare to execute the robot actions for the target zone that was found.
                targetZoneInsert = new ArrayList<>(Objects.requireNonNull(targetZoneCommands.get(targetZone)));
                break;
            }

            case "ACTIVATE_VUMARK_READER": {
                if (vumarkReader == null)
                    throw new AutonomousRobotException(TAG, "Vumark reader not initialized");

                vumarkReader.activateVumarkRecognition();
                break;
            }

            case "DEACTIVATE_VUMARK_READER": {
                if (vumarkReader != null) {
                    vumarkReader.deactivateVumarkRecognition();
                    vumarkReader = null;
                }
                break;
            }

            case "TEST_VUMARK_READER": {
                if (vumarkReader == null)
                    throw new AutonomousRobotException(TAG, "Vumark reader not initialized");

                // Get a head start on reading Vumarks instead of including a <SLEEP>1000</SLEEP> in the XML file.
                //**TODO hardcoded to RED_TOWER_GOAL -> need <vumark> element in RobotAction.xml
                Optional<Pose> vumark = vumarkReader.getMostRecentVumarkPose(VumarkReader.SupportedVumark.RED_TOWER_GOAL, 1000);
                if (!vumark.isPresent()) {
                    RobotLogCommon.d(TAG, "First attempt: most recent Vumark: not visible");
                    linearOpMode.telemetry.addData("First attempt: most recent Vumark ", "not visible");
                    linearOpMode.telemetry.update();
                }

                for (int i = 0; i < 20; i++) {
                    vumark = vumarkReader.getMostRecentVumarkPose(VumarkReader.SupportedVumark.RED_TOWER_GOAL, 1000);
                    if (!vumark.isPresent()) {
                        RobotLogCommon.d(TAG, "Most recent Vumark: not visible");
                        linearOpMode.telemetry.addData("Most recent Vumark ", "not visible");
                        linearOpMode.telemetry.update();
                    } else {
                        Pose robotPoseAtVumark = vumark.get();
                        RobotLogCommon.d(TAG, "Robot pose at Vumark " + VumarkReader.SupportedVumark.RED_TOWER_GOAL);
                        String poseString = String.format(Locale.US, "Pose x %.1f in., y %.1f in, angle %.1f deg.",
                                robotPoseAtVumark.x, robotPoseAtVumark.y, robotPoseAtVumark.r);
                        RobotLogCommon.d(TAG, poseString);

                        linearOpMode.telemetry.addData("Vumark ", poseString);
                        linearOpMode.telemetry.update();
                    }
                }

                RobotLogCommon.d(TAG, "Done with getMostRecentVumarkPose");

                // From a stationary position, get the median Vumark values over 11 samples.
                RobotLogCommon.d(TAG, "Calling getMedianVumarkPose");
                vumark = vumarkReader.getMedianVumarkPose(VumarkReader.SupportedVumark.RED_TOWER_GOAL, 1500, 11);
                if (!vumark.isPresent()) {
                    RobotLogCommon.d(TAG, "Vumark median: Vumark not visible");
                    linearOpMode.telemetry.addData("Vumark median:  ", "Vumark not visible");
                    linearOpMode.telemetry.update();
                } else {
                    Pose robotPoseAtVumark = vumark.get();
                    RobotLogCommon.d(TAG, "Median pose at Vumark " + VumarkReader.SupportedVumark.BLUE_TOWER_GOAL);
                    String poseString = String.format(Locale.US, "Pose x %.1f in., y %.1f in, angle %.1f deg.",
                            robotPoseAtVumark.x, robotPoseAtVumark.y, robotPoseAtVumark.r);
                    RobotLogCommon.d(TAG, poseString);

                    linearOpMode.telemetry.addData("Median pose at Vumark ", poseString);
                    linearOpMode.telemetry.update();
                }

                break;
            }

            case "ALIGN_BY_VUMARK": {
                String vumarkString = commandXPath.getString("vumark", true);
                VumarkReader.SupportedVumark vumark = VumarkReader.SupportedVumark.valueOf(vumarkString);
                double targetX = commandXPath.getDouble("target_x");
                double targetY = commandXPath.getDouble("target_y");
                double power = commandXPath.getDouble("power");
                double minPower = commandXPath.getDouble("minpower", 0.2);

                /*
                double targetClicks = commandXPath.getDouble("distance") * DriveTrain.CLICKS_PER_INCH;
                double marginClicks = commandXPath.getDouble("margin") * DriveTrain.CLICKS_PER_INCH; // stop when within {margin} clicks
                double power = commandXPath.getDouble("power");
                double minPower = commandXPath.getDouble("minpower", 0.2);
                Angle direction = AutoCommandXML.getAngle(commandXPath, "direction"); // direction angle; right is 0, up 90, left 180
                Angle targetHeading = AutoCommandXML.getAngle(commandXPath, "heading"); // robot's target heading angle while moving
                PIDController rPIDController = AutoCommandXML.getPIDController(commandXPath, targetHeading.getDegrees());
                 */

                // Break the movement between the current robot location and the target
                // robot location into two: a strafe and a move. Determine the direction
                // dynamically based on the sign of the difference between the two positions.
                Angle targetHeading = AutoCommandXML.getAngle(commandXPath, "heading", robot.imu.getHeading()); // robot's target heading angle while moving; default is current heading
                PIDController rPIDController = AutoCommandXML.getPIDController(commandXPath, targetHeading.getDegrees());

                //**TODO TEMP hardcode test case
                //Optional<Pose> vumarkPose = vumarkReader.getMostRecentVumarkPose(vumark, 1000);
                // In the real world you would have to translate the Vumark coordinates
                // to Cartesian depending on the Vumark target.
                Optional<Pose> vumarkPose = Optional.of(new Pose(10, 5, 0));
                //** if reading the Vumark once is not stable, try the next line --
                // Optional<Pose> vumarkPose = vumarkReader.getMedianVumarkPose(vumark, 1000, 5);
                if (!vumarkPose.isPresent()) {
                    RobotLogCommon.d(TAG, "Most recent Vumark: not visible");
                    //**TODO need default movements in case the Vumark is not visible.
                    // take from the XML file
                } else {
                    Pose robotPoseAtVumark = vumarkPose.get();
                    RobotLogCommon.d(TAG, "Robot pose at Vumark " + vumarkString);
                    String poseString = String.format(Locale.US, "Pose x %.1f in., y %.1f in, angle %.1f deg.",
                            robotPoseAtVumark.x, robotPoseAtVumark.y, robotPoseAtVumark.r);
                    RobotLogCommon.d(TAG, poseString);

                    //** Added robot motion - Trinity
                    //**TODO commented out because the robot went back and to the right instead of
                    // back and to the left for the test case.
                    /*
                    VumarkReader.RobotVector rv = vumarkReader.getDistanceAndAngleToTarget(robotPoseAtVumark, new Pose(targetX, targetY, 0));

                    Pose drivePose = new Pose();
                    drivePose.x = Math.sin(rv.angleToTarget.getRadians());
                    drivePose.y = Math.cos(rv.angleToTarget.getRadians());

                    // The result of a sin(angle) = distance (length of movement vector) NOT degrees. - Trinity
//                    RobotLogCommon.d(TAG, "Drive pose degrees x " + Math.toDegrees(drivePose.x) + ", y " + Math.toDegrees(drivePose.y));
                    RobotLogCommon.d(TAG, "drive pose: " + drivePose.toString());

                    robot.driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    robot.driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                    int currentClicks = 0;
                    int targetClicks = (int) Math.round(rv.distanceToTarget * DriveTrain.CLICKS_PER_INCH);

                    while (Math.abs(currentClicks - targetClicks) > 0) {
                        Angle actualHeading = robot.imu.getHeading();
                        currentClicks = Math.abs(robot.driveTrain.rb.getCurrentPosition());
                        drivePose.r = -rPIDController.getCorrectedOutput(actualHeading.getDegrees());
                        robot.driveTrain.drive(drivePose, power);
                    }
                    robot.driveTrain.stop();
                    */

                }
                break;
            }

            case "CLOSE_WEBCAM": {
                if (vuforiaLocalizer != null) {
                    RobotLogCommon.i(TAG, "Shutting down Vuforia");
                    Objects.requireNonNull(vuforiaLocalizer.getCamera()).close();
                    vuforiaLocalizer = null;
                }
                break;
            }

            case "EXECUTE_TARGET_ZONE_STEPS": {
                executeTargetZoneActions = true;
                break;
            }

            //**TODO Refactoring required - code duplicated in RobotActionCommon
            case "SLEEP": { //I want sleep :)
                int sleepValue = commandXPath.getInt("ms");
                RobotLogCommon.d(TAG, "Pause by " + sleepValue + " milliseconds");
                sleep(sleepValue);
                break;
            }

            //**TODO Refactoring required - code duplicated in RobotActionCommon
            case "BREAKPOINT": {
                while (!linearOpMode.gamepad1.a) {
                    sleep(1);
                }
                break;
            }

            case "GATE": {
                String stringState = commandXPath.getString("state");
                OldRingShooter.ServoState state = OldRingShooter.ServoState.REST;

                switch (stringState) {
                    case "down": {
                        state = OldRingShooter.ServoState.DOWN;
                        break;
                    }
                    case "rest": {
                        state = OldRingShooter.ServoState.REST;
                        break;
                    }
                    case "up": {
                        state = OldRingShooter.ServoState.UP;
                        break;
                    }

                    case "trigger": {
                        robot.shooter.triggerServo.setState("out");
                        sleep(300);
                        robot.shooter.triggerServo.setState("rest");
                        break;
                    }
                }
//                robot.shooter.setServoState(state);
                break;
            }

            //**TODO Refactoring required - code duplicated in RobotActionCommon
            case "TRIGGER": {
                robot.shooter.triggerServo.setState("out");
                sleep(1000);
                robot.shooter.triggerServo.setState("rest");

                break;
            }

            default: {
                throw new AutonomousRobotException(TAG, "No support for the command " + commandName);
            }
        }
    }

}

