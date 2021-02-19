package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
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
import java.util.logging.Level;

import org.firstinspires.ftc.teamcode.math.LCHSMath;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.xml.sax.SAXException;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathException;

import static android.os.SystemClock.sleep;

public class FTCAuto {

    private static final String TAG = "FTCAuto";

    private final LinearOpMode opMode;
    private final LCHSRobot robot;

    private final RobotConstants.Alliance alliance;
    private final RobotConstantsUltimateGoal.OpMode autoOpMode;
    private final String workingDirectory;

    private final VuforiaWebcam vuforiaWebcam;
    private VuforiaLocalizer vuforiaLocalizer;

    private final RobotActionXML actionXML;

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
    public FTCAuto(RobotConstantsUltimateGoal.OpMode autoOpMode, RobotConstants.Alliance alliance, LinearOpMode opMode)
            throws ParserConfigurationException, SAXException, XPathException, IOException, InterruptedException {

        RobotLogCommon.c(TAG, "FTCAuto constructor");

        this.autoOpMode = autoOpMode;
        this.alliance = alliance;
        if (alliance == RobotConstants.Alliance.UNKNOWN)
            throw new AutonomousRobotException(TAG, "Alliance is UNKNOWN");


        // Initialize the hardware.
        this.opMode = opMode;
        robot = new LCHSRobot(opMode);
        robot.initializeIMU();


        workingDirectory = WorkingDirectory.getWorkingDirectory();

        // Start the asynchronous initialization of Vuforia.
        RobotLogCommon.d(TAG, "start asynchronous initialization");
        vuforiaWebcam = new VuforiaWebcam(robot.webcam1Name);


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
        targetZoneCommands = targetZoneXML.getTargetZoneCommands(this.autoOpMode);

        // Wait for the asynchronous initialization of Vuforia to complete.
        RobotLogCommon.d(TAG, "wait for initialization");
        vuforiaLocalizer = vuforiaWebcam.waitForVuforiaInitialization();


        RobotLogCommon.c(TAG, "FTCAuto construction complete");
    }

    public void runRobot() throws XPathException, InterruptedException {

        RobotLogCommon.i(TAG, "At start");
        RobotLogCommon.i(TAG, "OpMode: " + autoOpMode + ", Alliance: " + alliance);

        // Safety check against ftc runtime initialization errors.
        // Make sure the opmode is still active.
        if (!opMode.opModeIsActive())
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
            opMode.telemetry.addData("FTCAuto", "COMPLETE");
            opMode.telemetry.update();
        }
    }

    //===============================================================================================
    //===============================================================================================

    // Using the XML elements ane attributes from the configuration file, RobotConfig.xml,
    // execute the command.
    private void doCommand(RobotActionXML.CommandXML commandXML) throws InterruptedException, XPathException {
        AutoCommandXML autoCommand = new AutoCommandXML(commandXML);

        RobotLogCommon.d(TAG, "Executing FTCAuto command " + autoCommand.getName());

        switch (autoCommand.getName()) {

            // Move by clicks
            case "MOVE": {
                double targetClicks = autoCommand.getDouble("distance"); // conversion is a pain so keep in clicks
                double marginClicks = autoCommand.getDouble("margin"); // stop when within {margin} clicks
                double power = autoCommand.getDouble("power");
                Angle direction = autoCommand.getAngle("direction"); // direction angle; right is 0, up 90, left 180
                Angle targetHeading = autoCommand.getAngle("heading"); // robot's target heading angle while moving
                PIDCoefficients kPID = autoCommand.getPID();
                PIDController rPIDController= new PIDController(kPID, targetHeading.getDegrees());

                Pose drivePose = new Pose();
                drivePose.x = Math.sin(direction.getRadians());
                drivePose.y = Math.cos(direction.getRadians());

                robot.driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                robot.driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                int currentClicks = Math.abs(robot.driveTrain.rb.getCurrentPosition());
                while (Math.abs(currentClicks - targetClicks) < marginClicks) {
                    Angle actualHeading = robot.imu.getHeading();
                    drivePose.r = rPIDController.getCorrectedOutput(actualHeading.getDegrees());

                    robot.driveTrain.drive(drivePose, power);
                }
                robot.driveTrain.stop();

                break;
            }

            // A normalized turn, i.e. a turn from 0 to +-180 degrees, which will always be the
            // shortest distance from the current heading to the desired heading.
            case "TURN": {
                Angle angle = autoCommand.getAngle("angle");
                double maxPower = autoCommand.getDouble("maxpower");
                double minPower = autoCommand.getDouble("minpower");
                double margin = autoCommand.getDouble("margin");
                PIDCoefficients kPID = autoCommand.getPID();
                PIDController pidController = new PIDController(kPID, angle.getDegrees());
                double currentDegrees = 0;

                RobotLogCommon.d(TAG, "Turn by " + angle + " degrees FTC" + ", turn type " + RobotConstants.TurnType.NORMALIZED);

                double error = Math.abs(currentDegrees - angle.getDegrees());
                while (error > margin) {
                    double pidCorrectedPower = pidController.getCorrectedOutput(robot.imu.getIntegratedHeading().getDegrees());
                    double power = LCHSMath.clipPower(pidCorrectedPower, minPower) * maxPower;
                    robot.driveTrain.drive(new Pose(0, 0, power));
                    error = Math.abs(currentDegrees - angle.getDegrees());
                }
                robot.driveTrain.stop();

                break;
            }


            // Use OpenCV to find the stack of rings and determine the Target Zone.
            case "RECOGNIZE_RINGS": {
                String imageProviderId = autoCommand.getStringInRange("ocv_image_provider", autoCommand.validRange("vuforia", "file"));
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
                opMode.telemetry.addData("Found ", targetZone);
                opMode.telemetry.update();

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
                int sleepValue = autoCommand.getInt("ms");
                RobotLogCommon.d(TAG, "Pause by " + sleepValue + " milliseconds");
                sleep(sleepValue);
            }
            default: {
                RobotLogCommon.d(TAG, "No support in the simulator for the command " + autoCommand.getName());
            }
        }
    }

}

