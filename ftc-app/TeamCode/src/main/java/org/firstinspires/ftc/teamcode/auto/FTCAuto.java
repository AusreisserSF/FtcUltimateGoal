package org.firstinspires.ftc.teamcode.auto;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.firstinspires.ftc.ftcdevcommon.android.WorkingDirectory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.auto.vision.FileImage;
import org.firstinspires.ftc.teamcode.auto.vision.ImageProvider;
import org.firstinspires.ftc.teamcode.auto.vision.RingParameters;
import org.firstinspires.ftc.teamcode.auto.vision.RingRecognition;
import org.firstinspires.ftc.teamcode.auto.vision.RingReturn;
import org.firstinspires.ftc.teamcode.auto.vision.T265Reader;
import org.firstinspires.ftc.teamcode.auto.vision.TowerGoalAlignment;
import org.firstinspires.ftc.teamcode.auto.vision.TowerParameters;
import org.firstinspires.ftc.teamcode.auto.vision.VuforiaImage;
import org.firstinspires.ftc.teamcode.auto.vision.VuforiaWebcam;
import org.firstinspires.ftc.teamcode.auto.vision.VumarkReader;
import org.firstinspires.ftc.teamcode.auto.xml.AutoCommandXML;
import org.firstinspires.ftc.teamcode.auto.xml.RingParametersXML;
import org.firstinspires.ftc.teamcode.auto.xml.RobotActionXML;
import org.firstinspires.ftc.teamcode.auto.xml.TargetZoneXML;
import org.firstinspires.ftc.teamcode.auto.xml.TowerParametersXML;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.LCHSMath;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.LCHSRobot;
import org.firstinspires.ftc.teamcode.robot.RingShooter;
import org.firstinspires.ftc.teamcode.robot.WobbleArm;
import org.opencv.android.OpenCVLoader;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import java.util.logging.Level;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathException;
import javax.xml.xpath.XPathFactory;

import static android.os.SystemClock.sleep;

public class FTCAuto {

    private static final String TAG = "FTCAuto";

    private final LinearOpMode linearOpMode;
    private final LCHSRobot robot;

    private final RobotConstants.Alliance alliance;
    private final RobotConstantsUltimateGoal.OpMode autoOpMode;
    private final String workingDirectory;

    private VuforiaWebcam vuforiaWebcam;
    private VuforiaLocalizer vuforiaLocalizer;
    private VumarkReader vumarkReader;
    private T265Reader t265Reader;

    private final RobotActionXML actionXML;
    private final XPathFactory xpathFactory = XPathFactory.newInstance();
    private final XPath xpath = xpathFactory.newXPath();

    // Ring recognition.
    private final RingParametersXML ringParametersXML;
    private final RingParameters ringParameters;
    private final RingRecognition ringRecognition;
    private final TowerParametersXML towerParametersXML;
    private final TowerParameters towerParameters;
    private final TowerGoalAlignment towerGoalAlignment;
    private final TargetZoneXML targetZoneXML;
    private final HashMap<RobotConstantsUltimateGoal.TargetZone, List<RobotActionXML.CommandXML>> targetZoneCommands;

    private RobotConstantsUltimateGoal.TargetZone targetZone;
    private List<RobotActionXML.CommandXML> targetZoneInsert = new ArrayList<>();
    private boolean executeTargetZoneCommands = false;

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
        Level minLogLevel = actionXML.getMinimumLoggingLevel();
        if (minLogLevel != null) // null means use the default
            RobotLogCommon.setMinimimLoggingLevel(minLogLevel);
        RobotLogCommon.c(TAG, "Minimum logging level " + RobotLogCommon.getMinimumLoggingLevel());

        // Start the asynchronous initialization of Vuforia.
        boolean initVuforia = actionXML.initializeVuforia();
        if (initVuforia) {
            RobotLogCommon.d(TAG, "Vuforia: start asynchronous initialization");
            vuforiaWebcam = new VuforiaWebcam(robot.webcam1Name);
        }

        // Read the ring stack image recognition parameters from an xml file.
        ringParametersXML = new RingParametersXML(xmlDirectory);
        ringParameters = ringParametersXML.getRingParameters();
        ringRecognition = new RingRecognition();

        // Read the tower goal image recognition parameters from an xml file.
        towerParametersXML = new TowerParametersXML(xmlDirectory);
        towerParameters = towerParametersXML.getTowerParameters();
        towerGoalAlignment = new TowerGoalAlignment();

        // Read the XML file with the target zones for all OpModes.
        // For the current OpMode get the commands for all three target zones.
        // We won't know which zone to go for until we've performed image recognition.
        targetZoneXML = new TargetZoneXML(xmlDirectory);
        targetZoneCommands = targetZoneXML.getTargetZoneCommands(this.autoOpMode);

        // Wait for the asynchronous initialization of Vuforia to complete.
        if (initVuforia) {
            RobotLogCommon.d(TAG, "wait for initialization");
            vuforiaLocalizer = vuforiaWebcam.waitForVuforiaInitialization();

            // Prepare to read Vumarks but don't start yet.
            vumarkReader = new VumarkReader(linearOpMode, vuforiaLocalizer,
                    new ArrayList<>(Collections.singletonList(VumarkReader.SupportedVumark.BLUE_TOWER_GOAL)));
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
        List<RobotActionXML.CommandXML> steps = actionXML.getOpModeCommands(autoOpMode.toString());
        try {
            for (RobotActionXML.CommandXML step : steps) {

                if (!executeTargetZoneCommands) // execute Target Zone specific steps now?
                    doCommand(step); // no, but doCommand may change that

                // Takes care of the case where the EXECUTE_TARGET_ZONE_STEPS
                // command is the last command for the opmode in RobotConfig.xml.
                if (executeTargetZoneCommands) { // any Target Zone specific steps?
                    // Yes, do all of those commands now.
                    for (RobotActionXML.CommandXML insertedStep : targetZoneInsert) {
                        if (insertedStep.getCommandId().equals("EXECUTE_TARGET_ZONE_STEPS"))
                            throw new AutonomousRobotException(TAG, "Nesting of EXECUTE_TARGET_ZONE_STEPS is not allowed");
                        doCommand(insertedStep);
                    }
                    targetZoneInsert.clear();
                    executeTargetZoneCommands = false;
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

    // Using the XML elements ane attributes from the configuration file, RobotConfig.xml,
    // execute the command.
    private void doCommand(RobotActionXML.CommandXML pCommand) throws InterruptedException, XPathException, IOException {

        // Set up XPath access to the current action command.
        XPathAccess commandXPath = new XPathAccess(xpath, pCommand.getCommandElement(), pCommand.getCommandId());
        String commandName = pCommand.getCommandId().toUpperCase();
        RobotLogCommon.d(TAG, "Executing FTCAuto command " + commandName);

        switch (commandName) {

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
                double lifterPower = commandXPath.getDouble("lifterPower", 0);
                double rampPercent = commandXPath.getDouble("rampPercent", 1);

                double dipVelocity = commandXPath.getDouble("dip", 500);
                double outtakePower = commandXPath.getDouble("outtakePower", 0.5);
                long timeStartedOuttaking = 0;
                long timeToOuttake = commandXPath.getInt("outtakeTime", 500);


                robot.ringShooter.intakeMotor.setPower(intakePower);
                robot.ringShooter.liftMotor.setPower(lifterPower);


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

                    if (timeStartedOuttaking == 0 && intakePower != 0 && robot.ringShooter.liftMotor.getVelocity() < dipVelocity) {
                        robot.ringShooter.intakeMotor.setPower(-outtakePower);
                        robot.ringShooter.liftMotor.setPower(-outtakePower);
                        timeStartedOuttaking = System.currentTimeMillis();
                    }

                    if (timeStartedOuttaking != 0 && System.currentTimeMillis() + timeToOuttake > timeStartedOuttaking) {
                        robot.ringShooter.intakeMotor.setPower(intakePower);
                        robot.ringShooter.liftMotor.setPower(intakePower);
                        timeStartedOuttaking = 0;
                    }
                    Angle actualHeading = robot.imu.getHeading();
                    currentClicks = Math.abs(robot.driveTrain.rb.getCurrentPosition());
                    drivePose.r = -rPIDController.getCorrectedOutput(actualHeading.getDegrees());
                    robot.driveTrain.drive(drivePose, power * rampPower);
                }

                robot.driveTrain.stop();
                robot.ringShooter.intakeMotor.setPower(0);
                robot.ringShooter.liftMotor.setPower(0);

                break;
            }
            // A normalized turn, i.e. a turn from 0 to +-180 degrees, which will always be the
            // shortest distance from the current heading to the desired heading.
            case "TURN": {
                Angle angle = AutoCommandXML.getAngle(commandXPath, "angle");
                double maxPower = commandXPath.getDouble("maxpower");
                double minPower = commandXPath.getDouble("minpower");
                double margin = commandXPath.getDouble("margin");
                PIDController pidController = AutoCommandXML.getPIDController(commandXPath, angle.getDegrees());

                double currentDegrees = robot.imu.getIntegratedHeading().getDegrees();

                RobotLogCommon.d(TAG, "Turn by " + angle + " degrees");

                double error = currentDegrees - angle.getDegrees();
                while (Math.abs(error) > margin) {
                    currentDegrees = robot.imu.getIntegratedHeading().getDegrees();
                    double pidCorrectedPower = -pidController.getCorrectedOutput(currentDegrees);
                    double power = LCHSMath.clipPower(pidCorrectedPower, minPower) * maxPower;
                    robot.driveTrain.drive(new Pose(0, 0, power));
                    error = currentDegrees - angle.getDegrees();
                }
                robot.driveTrain.stop();

                break;
            }

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

            case "WOBBLE_SERVO":{

                String stringState = commandXPath.getString("state");
                WobbleArm.ServoState state = WobbleArm.ServoState.REST;

                switch(stringState){
                    case "close":{
                        state = WobbleArm.ServoState.HOLD;
                        break;
                    }
                    case "neutral":{
                        state = WobbleArm.ServoState.REST;
                        break;
                    }
                    case "release":
                        state = WobbleArm.ServoState.RELEASE;
                        break;
                }
                robot.wobbleArm.setServoState(state);
                break;
            }


            case "ALIGN_TO_TOWER": {
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
                double angleAdjustment = towerGoalAlignment.getAngleToTowerGoal(imageProvider, towerParameters);

                RobotLogCommon.d(TAG, "Angle adjustment " + angleAdjustment);
                linearOpMode.telemetry.addData("Angle adjustment ", angleAdjustment);
                linearOpMode.telemetry.update();
                break;
            }

            case "SHOOT": {
                double shootVelocity = commandXPath.getDouble("shootVelocity");
                double intakeVelocity = commandXPath.getDouble("intakeVelocity");
                int waitTime = commandXPath.getInt("waitTime");
                int dip = commandXPath.getInt("shootVelocityDip");

                //optional parameters
                double lifterVelocity = commandXPath.getDouble("lifterVelocity", 0);
                int maxShotCount = commandXPath.getInt("maxShotCount",3);
                boolean powerShot = commandXPath.getBoolean("powerShot", false);

                if (powerShot){
                    robot.ringShooter.intakeMotor.setVelocity(intakeVelocity);
                    robot.ringShooter.liftMotor.setVelocity(lifterVelocity);
                }

                robot.ringShooter.shootMotor.setVelocity(shootVelocity);
                double currentVelocity = robot.ringShooter.shootMotor.getVelocity();
                while (currentVelocity < shootVelocity) {
                    currentVelocity = robot.ringShooter.shootMotor.getVelocity();
                    sleep(20);
                }

                sleep(500);

                if (!powerShot){
                    robot.ringShooter.intakeMotor.setVelocity(intakeVelocity);
                    robot.ringShooter.liftMotor.setVelocity(lifterVelocity);
                }

                int shotCount = 0;
                long timeout = System.currentTimeMillis() + waitTime;
                while (System.currentTimeMillis() < timeout) {
                    currentVelocity = robot.ringShooter.shootMotor.getVelocity();
                    if (currentVelocity < dip) {
                        shotCount++;
                        RobotLogCommon.d(TAG, "Presume shot taken; dip in shooter motor velocity to " + currentVelocity);
                        RobotLogCommon.d(TAG, "Shot count " + shotCount);
                        if (shotCount == maxShotCount)
                            break;

                        robot.ringShooter.intakeMotor.setVelocity(0);
                        robot.ringShooter.liftMotor.setVelocity(0);
                        sleep(100);

                        // Get back up to speed
                        currentVelocity = robot.ringShooter.shootMotor.getVelocity();
                        while (currentVelocity < shootVelocity) {
                            currentVelocity = robot.ringShooter.shootMotor.getVelocity();
                            sleep(20);
                        }

                        robot.ringShooter.intakeMotor.setVelocity(intakeVelocity);
                        robot.ringShooter.liftMotor.setVelocity(lifterVelocity);
                    }
                }

                robot.ringShooter.shootMotor.setVelocity(0);
                robot.ringShooter.intakeMotor.setVelocity(0);
                robot.ringShooter.liftMotor.setVelocity(0);
                break;
            }

            case "INTAKE": {
                double dipVelocity = commandXPath.getDouble("dip", 100);
                double intakePower = commandXPath.getDouble("intakePower");
                double outtakePower = commandXPath.getDouble("outtakePower", 0.5);
                int intakeTime = commandXPath.getInt("intakeTime");
                robot.ringShooter.intakeMotor.setPower(intakePower);
                robot.ringShooter.liftMotor.setPower(intakePower);

                double timeWhenWeAreDone = System.currentTimeMillis() + intakeTime;

                while (System.currentTimeMillis() < timeWhenWeAreDone) {
                    if (robot.ringShooter.liftMotor.getVelocity() < dipVelocity) {
                        robot.ringShooter.intakeMotor.setPower(-outtakePower);
                        robot.ringShooter.liftMotor.setPower(-outtakePower);
                        sleep(500);
                        robot.ringShooter.intakeMotor.setPower(intakePower);
                        robot.ringShooter.liftMotor.setPower(intakePower);
                    }
                }
                robot.ringShooter.intakeMotor.setPower(0);
                robot.ringShooter.intakeMotor.setPower(0);

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
                Optional<Pose> vumark = vumarkReader.getMostRecentVumarkPose(VumarkReader.SupportedVumark.BLUE_TOWER_GOAL, 1000);;
                for (int i = 0; i < 20; i++) {
                    vumark = vumarkReader.getMostRecentVumarkPose(VumarkReader.SupportedVumark.BLUE_TOWER_GOAL, 1000);
                    if (!vumark.isPresent()) {
                        RobotLogCommon.d(TAG, "Most recent Vumark: not visible");
                        linearOpMode.telemetry.addData("\"Most recent Vumark ", "not visible");
                        linearOpMode.telemetry.update();
                    } else {
                        Pose robotPoseAtVumark = vumark.get();
                        RobotLogCommon.d(TAG, "Robot pose at Vumark " + VumarkReader.SupportedVumark.BLUE_TOWER_GOAL);
                        String poseString = String.format(Locale.US, "Pose x %.1f in., y %.1f in, angle %.1f deg.",
                                robotPoseAtVumark.x, robotPoseAtVumark.y, robotPoseAtVumark.r);
                        RobotLogCommon.d(TAG, poseString);

                        linearOpMode.telemetry.addData("Vumark ", poseString);
                        linearOpMode.telemetry.update();
                    }
                    sleep(500);

                    // From a stationary position, get the median Vumark values over 11 samples.
                    vumark = vumarkReader.getMedianVumarkPose(VumarkReader.SupportedVumark.BLUE_TOWER_GOAL,1500, 11);
                    if (!vumark.isPresent()) {
                        RobotLogCommon.d(TAG, "Vumark median: Vumark not visible");
                        linearOpMode.telemetry.addData("Vumark median:  ", "Vumark not visible");
                        linearOpMode.telemetry.update();
                    }
                    else {
                        Pose robotPoseAtVumark = vumark.get();
                        RobotLogCommon.d(TAG, "Median pose at Vumark " + VumarkReader.SupportedVumark.BLUE_TOWER_GOAL);
                        String poseString = String.format(Locale.US, "Pose x %.1f in., y %.1f in, angle %.1f deg.",
                                robotPoseAtVumark.x, robotPoseAtVumark.y, robotPoseAtVumark.r);
                        RobotLogCommon.d(TAG, poseString);

                        linearOpMode.telemetry.addData("Median pose at Vumark ", poseString);
                        linearOpMode.telemetry.update();
                    }
                }
                break;
            }


            case "MOVE_WITH_T265_WITHOUT_READER": {
                robot.initializeT265Camera();

                Pose targetPose = AutoCommandXML.getPose(commandXPath, "target");
                Pose marginPose = AutoCommandXML.getPose(commandXPath, "margin");

                double power = commandXPath.getDouble("power", 1.0);
                PIDController xPIDController = AutoCommandXML.getPIDController(commandXPath, targetPose.x, "x");
                PIDController yPIDController = AutoCommandXML.getPIDController(commandXPath, targetPose.y, "y");
                PIDController rPIDController = AutoCommandXML.getPIDController(commandXPath, targetPose.r, "r");

                T265Camera.CameraUpdate slamraUpdate = robot.slamra.getLastReceivedCameraUpdate();
                Pose2d t265Pose = slamraUpdate.pose;
                Pose currentPose = new Pose(t265Pose.getTranslation().getX(), t265Pose.getTranslation().getY(), robot.imu.getIntegratedHeading().getDegrees());
                Pose diffPose = currentPose.subtract(targetPose);

                linearOpMode.telemetry.setAutoClear(true);
                while (Math.abs(diffPose.x) > marginPose.x || Math.abs(diffPose.y) > marginPose.y || Math.abs(diffPose.r) > marginPose.r) {
                    Pose drivePose = new Pose();
                    drivePose.x = LCHSMath.clipPower(-xPIDController.getCorrectedOutput(currentPose.x));
                    drivePose.y = LCHSMath.clipPower(-yPIDController.getCorrectedOutput(currentPose.y));
                    drivePose.r = LCHSMath.clipPower(-rPIDController.getCorrectedOutput(currentPose.r));
                    robot.driveTrain.drive(drivePose, power);

                    linearOpMode.telemetry.addData("currentPose", currentPose.toString(",\t"));
                    linearOpMode.telemetry.addData("diffPose", diffPose.toString(",\t"));
                    linearOpMode.telemetry.addData("drivePose", drivePose.toString(",\t"));
                    linearOpMode.telemetry.update();

                    slamraUpdate = robot.slamra.getLastReceivedCameraUpdate();
                    t265Pose = slamraUpdate.pose;
                    currentPose = new Pose(t265Pose.getTranslation().getX(), t265Pose.getTranslation().getY(), robot.imu.getIntegratedHeading().getDegrees());
                    diffPose = currentPose.subtract(targetPose);
                    sleep(20);
                }

                robot.slamra.stop();

                break;
            }


            case "MOVE_WITH_T265": {
                Pose targetPose = AutoCommandXML.getPose(commandXPath, "target");
                Pose marginPose = AutoCommandXML.getPose(commandXPath, "margin");
                int newPoseTimeout = commandXPath.getInt("poseTimeout", 1000);
                double power = commandXPath.getDouble("power", 1.0);
                PIDController xPIDController = AutoCommandXML.getPIDController(commandXPath, targetPose.x, "x");
                PIDController yPIDController = AutoCommandXML.getPIDController(commandXPath, targetPose.y, "y");
                PIDController rPIDController = AutoCommandXML.getPIDController(commandXPath, targetPose.r, "r");

                if (t265Reader == null)
                    throw new AutonomousRobotException(TAG, "T265 reader is null");

                // Increase the covariance value to trust encoder odometry less when fusing encoder measurements with VSLAM
                double encoderMeasurementCovariance = commandXPath.getDouble("encoder_measurement_covariance", 0.8);
                t265Reader = new T265Reader(robot.hardwareMap, linearOpMode);
                t265Reader.activateT265Localization(encoderMeasurementCovariance);

                Optional<Pose> t265Pose = t265Reader.getT265Pose(newPoseTimeout);
                if (t265Pose.isPresent()) {

                    Pose currentPose = new Pose(t265Pose.get().x, t265Pose.get().y, robot.imu.getIntegratedHeading().getDegrees());
                    Pose diffPose = currentPose.subtract(targetPose);

                    linearOpMode.telemetry.setAutoClear(true);
                    while (Math.abs(diffPose.x) > marginPose.x || Math.abs(diffPose.y) > marginPose.y || Math.abs(diffPose.r) > marginPose.r) {
                        Pose drivePose = new Pose();
                        drivePose.x = LCHSMath.clipPower(-xPIDController.getCorrectedOutput(currentPose.x));
                        drivePose.y = LCHSMath.clipPower(-yPIDController.getCorrectedOutput(currentPose.y));
                        drivePose.r = LCHSMath.clipPower(-rPIDController.getCorrectedOutput(currentPose.r));
                        robot.driveTrain.drive(drivePose, power);

                        linearOpMode.telemetry.addData("t265pose", t265Pose.get().toString(",\t"));
                        linearOpMode.telemetry.addData("currentPose", currentPose.toString(",\t"));
                        linearOpMode.telemetry.addData("diffPose", diffPose.toString(",\t"));
                        linearOpMode.telemetry.addData("drivePose", drivePose.toString(",\t"));
                        linearOpMode.telemetry.update();

                        Optional<Pose> newT265Pose = t265Reader.getT265Pose(newPoseTimeout);
                        if (newT265Pose.isPresent())
                            t265Pose = newT265Pose;
                        currentPose = new Pose(t265Pose.get().x, t265Pose.get().y, robot.imu.getIntegratedHeading().getDegrees());
                        diffPose = currentPose.subtract(targetPose);
                        sleep(20);
                    }
                } else {
                    RobotLogCommon.d(TAG, "T265 reading not present");
                    linearOpMode.telemetry.addData("T265", "reading not present :(");
                    linearOpMode.telemetry.update();
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

            case "ACTIVATE_T265_READER": {
                if (t265Reader != null)
                    throw new AutonomousRobotException(TAG, "T265 reader already activated");

                double encoderMeasurementCovariance = commandXPath.getDouble("encoder_measurement_covariance", 0.8);

                t265Reader = new T265Reader(robot.hardwareMap, linearOpMode);
                // Increase the covariance value to trust encoder odometry less when fusing encoder measurements with VSLAM
                t265Reader.activateT265Localization(encoderMeasurementCovariance);
                break;
            }

            case "DEACTIVATE_T265_READER": {
                if (t265Reader != null) {
                    t265Reader.deactivateT265Localization();
                    t265Reader = null;
                }
                break;
            }

            case "TEST_T265_READER": {
                if (t265Reader == null)
                    throw new AutonomousRobotException(TAG, "T265 reader not initialized");

                Optional<Pose> t265Pose;
                for (int i = 0; i < 20; i++) {
                    t265Pose = t265Reader.getT265Pose(1000);
                    if (!t265Pose.isPresent()) {
                        RobotLogCommon.d(TAG, "T265: data not available");
                        linearOpMode.telemetry.addData("T265 ", "data not available");
                        linearOpMode.telemetry.update();
                    } else {
                        Pose robotPoseFromT265 = t265Pose.get();
                        String poseString = String.format(Locale.US, "Pose x %.1f in., y %.1f in, angle %.1f deg.",
                                robotPoseFromT265.x, robotPoseFromT265.y, robotPoseFromT265.r);
                        RobotLogCommon.d(TAG, "Robot pose from T265 " + poseString);

                        linearOpMode.telemetry.addData("T265 ", poseString);
                        linearOpMode.telemetry.update();
                    }
                    sleep(500);
                }
                break;
            }

            case "EXECUTE_TARGET_ZONE_STEPS": {
                executeTargetZoneCommands = true;
                break;
            }

            case "SLEEP": { //I want sleep :)
                int sleepValue = commandXPath.getInt("ms");
                RobotLogCommon.d(TAG, "Pause by " + sleepValue + " milliseconds");
                sleep(sleepValue);
                break;
            }

            case "BREAKPOINT": {
                while (!linearOpMode.gamepad1.a) {
                    sleep(1);
                }
                break;
            }

            case "GATE": {
                String stringState = commandXPath.getString("state");
                RingShooter.ServoState state = RingShooter.ServoState.REST;

                switch(stringState){
                    case "down":{
                        state = RingShooter.ServoState.DOWN;
                        break;
                    }
                    case "rest":{
                        state = RingShooter.ServoState.REST;
                        break;
                    }
                    case "up":
                        state = RingShooter.ServoState.UP;
                        break;
                }
                robot.ringShooter.setServoState(state);
                break;
            }

            default: {
                throw new AutonomousRobotException(TAG, "No support for the command " + commandName);

            }
        }
    }

}

