package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.Pair;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.firstinspires.ftc.ftcdevcommon.android.WorkingDirectory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.robot.LCHSRobot;
import org.firstinspires.ftc.teamcode.auto.vision.*;
import org.firstinspires.ftc.teamcode.auto.xml.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.logging.Level;

import org.firstinspires.ftc.teamcode.math.LCHSMath;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.RingShooter;
import org.firstinspires.ftc.teamcode.robot.WobbleArm;
import org.opencv.android.OpenCVLoader;
import org.xml.sax.SAXException;

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

    private final RobotConfigXML configXML;
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
        // RobotLogCommon outputs to text file on robot.
        // RobotLog can output to live logcat on Android Studio.
        RobotLog.dd(TAG, "FTCAuto Constructor");

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

        // Read the robot configuration file.
        String xmlDirectory = workingDirectory + RobotConstants.xmlDir;
        configXML = new RobotConfigXML(xmlDirectory);

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
            vumarkReader = new VumarkReader(linearOpMode, vuforiaLocalizer);
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

        XPathAccess configXPath; // XPath access to elements in the configuration file

        //** As needed in the commands below. Example:
        //configXPath = configXML.getPath("WOBBLE_SERVO");
        //String upDown = configXPath.getString("position");

        // Set up XPath access to the current action command.
        XPathAccess commandXPath = new XPathAccess(xpath, pCommand.getCommandElement(), pCommand.getCommandId());
        String commandName = pCommand.getCommandId().toUpperCase();
        RobotLogCommon.d(TAG, "Executing FTCAuto command " + commandName);

        switch (commandName) {

            case "MOVE": {
                double targetClicks = commandXPath.getDouble("distance") * robot.driveTrain.CLICKS_PER_INCH;
                double marginClicks = commandXPath.getDouble("margin") * robot.driveTrain.CLICKS_PER_INCH; // stop when within {margin} clicks
                double power = commandXPath.getDouble("power");
                double minPower = commandXPath.getDouble("minpower", 0.2);
                Angle direction = AutoCommandXML.getAngle(commandXPath, "direction"); // direction angle; right is 0, up 90, left 180
                Angle targetHeading = AutoCommandXML.getAngle(commandXPath, "heading"); // robot's target heading angle while moving
                PIDController rPIDController = AutoCommandXML.getPIDController(commandXPath, targetHeading.getDegrees());

                // optional parameters
                double intakePower = commandXPath.getDouble("intakePower", 0);
                double lifterPower = commandXPath.getDouble("lifterPower", 0);
                double rampPercent = commandXPath.getDouble("rampPercent", 1);

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

            case "PLACE_WOBBLE": {
                int waitTime = commandXPath.getInt("waitTime");
                robot.wobbleArm.setFlipState(WobbleArm.FlipState.OUT);
                robot.wobbleArm.waitForFlip();
                robot.wobbleArm.setServoState(WobbleArm.ServoState.RELEASE);
                sleep(waitTime);
                robot.wobbleArm.setServoState(WobbleArm.ServoState.REST);
                robot.wobbleArm.setFlipState(WobbleArm.FlipState.IN);
                break;
            }

            case "CLOSE_WOBBLE_SERVO": {
                robot.wobbleArm.setServoState(WobbleArm.ServoState.HOLD);
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
                double intakePower = commandXPath.getDouble("intakePower");
                int waitTime = commandXPath.getInt("waitTime");
                int margin = commandXPath.getInt("velocityMargin");

                //optional parameters
                double lifterPower = commandXPath.getDouble("lifterPower", 0);

                robot.ringShooter.shootMotor.setVelocity(shootVelocity);
                double currentVelocity = robot.ringShooter.shootMotor.getVelocity();
                while (currentVelocity < shootVelocity - margin && currentVelocity > shootVelocity + margin) {
                    currentVelocity = robot.ringShooter.shootMotor.getVelocity();
                    sleep(20);
                }
                sleep(2500);

                robot.ringShooter.intakeMotor.setPower(intakePower);
                robot.ringShooter.liftMotor.setPower(lifterPower);
                sleep(waitTime);
                robot.ringShooter.shootMotor.setVelocity(0);
                robot.ringShooter.intakeMotor.setPower(0);
                robot.ringShooter.liftMotor.setPower(0);

                break;
            }


            case "INTAKE": {
                int power = commandXPath.getInt("power");
                int time = commandXPath.getInt("time");
                robot.ringShooter.intakeMotor.setPower(power);
                sleep(time);
                robot.ringShooter.intakeMotor.setPower(0);
                break;
            }

            case "INTAKE_LIFTER": {

                int intakePower = commandXPath.getInt("intakePower");
                int intakeTime = commandXPath.getInt("intakeTime");
                robot.ringShooter.intakeMotor.setPower(intakePower);
                robot.ringShooter.liftMotor.setPower(intakePower);
                sleep(intakeTime);
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

                Optional<Pair<Pose, String>> vumark;
                for (int i = 0; i < 20; i++) {
                    vumark = vumarkReader.getVumarkPose(1000);
                    if (!vumark.isPresent()) {
                        RobotLogCommon.d(TAG, "Vumark is not visible");
                        linearOpMode.telemetry.addData("Vumark ", "not visible");
                        linearOpMode.telemetry.update();
                    } else {
                        Pair<Pose, String> robotPoseAtVumark = vumark.get();
                        RobotLogCommon.d(TAG, "Robot pose at Vumark " + robotPoseAtVumark.second);
                        RobotLogCommon.d(TAG, "Pose x " + robotPoseAtVumark.first.x +
                                ", y " + robotPoseAtVumark.first.y +
                                ", angle " + robotPoseAtVumark.first.r);

                        linearOpMode.telemetry.addData("Vumark ", robotPoseAtVumark.first.x +
                                ", " + robotPoseAtVumark.first.y + ", " + robotPoseAtVumark.first.r);
                        linearOpMode.telemetry.update();
                    }
                    sleep(500);
                }
                break;
            }

            case "MOVE_WITH_VUMARK": {
                Pose targetPose = AutoCommandXML.getPose(commandXPath, "target");
                Pose marginPose = AutoCommandXML.getPose(commandXPath, "margin");
                double power = commandXPath.getDouble("power", 1.0);
                double minPower = commandXPath.getDouble("minpower", 0.2);
                PIDController xPIDController = AutoCommandXML.getPIDController(commandXPath, targetPose.x, "x");
                PIDController yPIDController = AutoCommandXML.getPIDController(commandXPath, targetPose.y, "y");
                PIDController rPIDController = AutoCommandXML.getPIDController(commandXPath, targetPose.r, "r");

                if (vumarkReader == null)
                    throw new AutonomousRobotException(TAG, "Vumark reader not initialized");

                Optional<Pair<Pose, String>> vumark = vumarkReader.getVumarkPose(1000);
                if (vumark.isPresent()) {

                    // ignore squiggly; vuforia's x is robot's y
                    Pose currentPose = new Pose(vumark.get().first.y, vumark.get().first.x, vumark.get().first.r);
                    Pose diffPose = currentPose.subtract(targetPose);

                    while (Math.abs(diffPose.x) > marginPose.x && Math.abs(diffPose.y) > marginPose.y && Math.abs(diffPose.r) > marginPose.r) {
                        Pose drivePose = new Pose();
                        drivePose.x = LCHSMath.clipPower(xPIDController.getCorrectedOutput(diffPose.x), minPower);
                        drivePose.y = LCHSMath.clipPower(yPIDController.getCorrectedOutput(diffPose.y), minPower);
                        drivePose.r = LCHSMath.clipPower(-rPIDController.getCorrectedOutput(currentPose.r));
                        robot.driveTrain.drive(drivePose, power);

                        RobotLogCommon.d(TAG, currentPose.toString());
                        linearOpMode.telemetry.addData("vumark", currentPose.toString());
                        linearOpMode.telemetry.update();

                        Optional<Pair<Pose, String>> newVumark = vumarkReader.getVumarkPose(1000);
                        if (newVumark.isPresent())
                            vumark = newVumark;
                        currentPose = new Pose(vumark.get().first.y, vumark.get().first.x, vumark.get().first.r);
                        diffPose = currentPose.subtract(targetPose);
                    }
                } else {
                    RobotLogCommon.d(TAG, "Vumark not present");
                    linearOpMode.telemetry.addData("Vumark", "not present :(");
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

            case "EXECUTE_TARGET_ZONE_STEPS": {
                executeTargetZoneCommands = true;
                break;
            }

            case "SLEEP": {
                int sleepValue = commandXPath.getInt("ms");
                RobotLogCommon.d(TAG, "Pause by " + sleepValue + " milliseconds");
                sleep(sleepValue);
                break;
            }

            case "BREAK_POINT": {
                while (!linearOpMode.gamepad1.a) {
                    sleep(1);
                }
                break;
            }

            case "GATE_UP": {
                RingShooter.ServoState servoState = RingShooter.ServoState.UP;
                robot.ringShooter.setServoState(servoState);

                break;
            }

            case "GATE_DOWN": {
                RingShooter.ServoState servoState = RingShooter.ServoState.DOWN;
                robot.ringShooter.setServoState(servoState);

                break;
            }

            default: {
                throw new AutonomousRobotException(TAG, "No support for the command " + commandName);

            }
        }
    }

}

