package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.firstinspires.ftc.ftcdevcommon.android.WorkingDirectory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.LCHSHardwareMap;
import org.firstinspires.ftc.teamcode.auto.vision.*;
import org.firstinspires.ftc.teamcode.auto.xml.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.logging.Level;

import org.xml.sax.SAXException;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathException;
import javax.xml.xpath.XPathFactory;

public class FTCAuto {

    private static final String TAG = "FTCAuto";

    private final LinearOpMode linearOpMode;
    private final LCHSHardwareMap robot;

    private final RobotConstants.Alliance alliance;
    private final RobotConstantsUltimateGoal.OpMode autoOpMode;
    private final String workingDirectory;

    private final VuforiaWebcam autoVuforia;
    private VuforiaLocalizer vuforiaLocalizer;
    private final WebcamName webcam1Name;

    private final RobotActionXML actionXML;
    private final XPathFactory xpathFactory = XPathFactory.newInstance();
    private final XPath xpath = xpathFactory.newXPath();

    // Ring recognition.
    private final RingParametersXML ringParametersXML;
    private final RingParameters ringParameters;
    private final RingRecognition ringRecognition;
    private final TargetZoneXML targetZoneXML;
    private final HashMap<RobotConstantsUltimateGoal.TargetZone, List<RobotActionXML.CommandXML>> targetZoneCommands;

    private RobotConstantsUltimateGoal.TargetZone targetZone;
    private List<RobotActionXML.CommandXML> targetZoneInsert = new ArrayList<>();
    private boolean executeTargetZoneCommands = false;

    // Main class for the autonomous run.
    public FTCAuto(RobotConstantsUltimateGoal.OpMode pOpMode, RobotConstants.Alliance pAlliance, LinearOpMode pLinear)
            throws ParserConfigurationException, SAXException, XPathException, IOException, InterruptedException {

        RobotLogCommon.c(TAG, "FTCAuto constructor");

        autoOpMode = pOpMode;
        alliance = pAlliance;
        if (alliance == RobotConstants.Alliance.UNKNOWN)
            throw new AutonomousRobotException(TAG, "Alliance is UNKNOWN");

        workingDirectory = WorkingDirectory.getWorkingDirectory();

        // Initialize the hardware.
        linearOpMode = pLinear;
        robot = new LCHSHardwareMap(pLinear.hardwareMap, true);
        webcam1Name = robot.webcam1Name;

        // Start the asynchronous initialization of Vuforia.
        RobotLogCommon.d(TAG, "Vuforia: start asynchronous initialization");
        autoVuforia = new VuforiaWebcam(webcam1Name);

        // Read the robot action file for all opmodes.
        String xmlDirectory = workingDirectory + RobotConstants.xmlDir;
        actionXML = new RobotActionXML(xmlDirectory);
        Level minLogLevel = actionXML.getMinimumLoggingLevel();
        if (minLogLevel != null) // null means use the default
            RobotLogCommon.setMinimimLoggingLevel(minLogLevel);
        RobotLogCommon.c(TAG, "Minimum logging level " + RobotLogCommon.getMinimumLoggingLevel());

        // Read the image recognition parameters from an xml file.
        ringParametersXML = new RingParametersXML(xmlDirectory);
        ringParameters = ringParametersXML.getRingParameters();
        ringRecognition = new RingRecognition();

        // Read the XML file with the target zones for all OpModes.
        // For the current OpMode get the commands for all three target zones.
        // We won't know which zone to go for until we've performed image recognition.
        targetZoneXML = new TargetZoneXML(xmlDirectory);
        targetZoneCommands = targetZoneXML.getTargetZoneCommands(autoOpMode);

        // Wait for the asynchronous initialization of Vuforia to complete.
        RobotLogCommon.d(TAG, "Vuforia: wait for initialization");
        vuforiaLocalizer = autoVuforia.waitForVuforiaInitialization();

        RobotLogCommon.c(TAG, "FTCAuto construction complete");
    }

    public void runRobot() throws XPathException, InterruptedException {

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

            RobotLogCommon.i(TAG, "Exiting FTCAuto");
            linearOpMode.telemetry.addData("FTCAuto", "COMPLETE");
            linearOpMode.telemetry.update();
        }
    }

    //===============================================================================================
    //===============================================================================================

    // Using the XML elements ane attributes from the configuration file, RobotConfig.xml,
    // execute the command.
    private void doCommand(RobotActionXML.CommandXML pCommand) throws InterruptedException, XPathException {
        XPathAccess commandXPath = new XPathAccess(xpath, pCommand.getCommandElement(), pCommand.getCommandId());

        String commandName = pCommand.getCommandId().toUpperCase();
        RobotLogCommon.d(TAG, "Executing FTCAuto command " + commandName);

        switch (commandName) {

            // Just as lineTo and strafeTo are synonymous in Road Runner
            // make STRAIGHT_LINE and STRAFE_BY synonymous here. But
            // the use of STRAFE_BY is clearest when the angle is not
            // divisible by 90. Otherwise use STRAFE_RIGHT and STRAFE_LEFT
            // below.
            case "STRAIGHT_LINE":
            case "STRAFE_BY": {
                double distance = commandXPath.getDouble("distance");
                double power = commandXPath.getDouble("power");
                double angle = commandXPath.getDouble("angle");

                //** motion.moveRobotStraight(distance, power, angle);
                break;
            }

            // Specialization of STRAIGHT_LINE.
            case "FORWARD": {
                double distance = commandXPath.getDouble("distance");
                double power = commandXPath.getDouble("power");

                //** motion.moveRobotStraight(distance, power, 0.0);
                break;
            }

            // Specialization of STRAIGHT_LINE.
            case "BACK": {
                double distance = commandXPath.getDouble("distance");
                double power = commandXPath.getDouble("power");

                //** motion.moveRobotStraight(distance, power, -180.0);
                break;
            }

            // Specialization of STRAIGHT_LINE.
            case "STRAFE_LEFT": {
                double distance = commandXPath.getDouble("distance");
                double power = commandXPath.getDouble("power");

                //** motion.moveRobotStraight(distance, power, 90.0);
                break;
            }

            // Specialization of STRAIGHT_LINE.
            case "STRAFE_RIGHT": {
                double distance = commandXPath.getDouble("distance");
                double power = commandXPath.getDouble("power");

                //** motion.moveRobotStraight(distance, power, -90.0);
                break;
            }

            // A normalized turn, i.e. a turn from 0 to +-180 degrees, which will always be the
            // shortest distance from the current heading to the desired heading.
            case "TURN": {
                double angle = commandXPath.getDouble("angle");
                double power = commandXPath.getDouble("power");

                RobotLogCommon.d(TAG, "Turn by " + angle + " degrees FTC" + ", turn type " + RobotConstants.TurnType.NORMALIZED);
                //** motion.rotateRobot(angle, power, RobotConstants.TurnType.NORMALIZED);
                break;
            }

            // A turn in the requested direction from 0 to +-360 degrees.
            case "TURN_UNNORMALIZED": {
                double angle = commandXPath.getDouble("angle");
                double power = commandXPath.getDouble("power");

                RobotLogCommon.d(TAG, "Turn by " + angle + " degrees FTC" + ", turn type " + RobotConstants.TurnType.UNNORMALIZED);
                //** motion.rotateRobot(angle, power, RobotConstants.TurnType.UNNORMALIZED);
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

            case "CLOSE_WEBCAM": {
                if (vuforiaLocalizer != null) {
                    RobotLogCommon.i(TAG, "Shutting down Vuforia");
                    Objects.requireNonNull(vuforiaLocalizer.getCamera()).close();
                    vuforiaLocalizer = null;
                }
                break;
            }

            case "DELIVER_WOBBLE_GOAL": {
                //** pointScoring.deliverWobbleGoal();
                break;
            }

            case "PLACE_RING_IN_LOW_GOAL": {
                //** pointScoring.placeRingInLowGoal();
                break;
            }

            case "LAUNCH_RING_AT_MEDIUM_GOAL": {
                //** pointScoring.launchRingAtMediumGoal();
                break;
            }

            case "LAUNCH_RING_AT_HIGH_GOAL": {
                //** pointScoring.launchRingAtHighGoal();
                break;
            }

            case "LAUNCH_RING_AT_POWER_SHOT_GOAL": {
                //** pointScoring.launchRingAtPowerShotGoal();
                break;
            }

            case "EXECUTE_TARGET_ZONE_STEPS": {
                executeTargetZoneCommands = true;
                break;
            }

            case "SLEEP": {
                int sleepValue = commandXPath.getInt("ms");
                RobotLogCommon.d(TAG, "Pause by " + sleepValue + " milliseconds");
                //** pauseRobot(sleepValue);
            }
            default: {
                RobotLogCommon.d(TAG, "No support in the simulator for the command " + commandName);
            }
        }
    }

}

