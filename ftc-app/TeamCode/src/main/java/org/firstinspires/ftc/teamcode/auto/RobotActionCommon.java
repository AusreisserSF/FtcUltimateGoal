package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.RobotXMLElement;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.firstinspires.ftc.teamcode.auto.xml.AutoCommandXML;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.LCHSMath;
import org.firstinspires.ftc.teamcode.math.PIDController;
import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.robot.DriveTrain;
import org.firstinspires.ftc.teamcode.robot.LCHSRobot;

import java.io.IOException;
import java.util.List;
import javax.xml.xpath.XPathException;
import static android.os.SystemClock.sleep;

public class RobotActionCommon {

    private static final String TAG = "RobotActionCommon";
 
    private final LCHSRobot robot;
    private final LinearOpMode linearOpMode;

    // For both Autonomous and TeleOp: class that executes actions from RobotAction.xml. 
    public RobotActionCommon(LCHSRobot pRobot, LinearOpMode pLinearOpMode) {
        robot = pRobot;
        linearOpMode = pLinearOpMode;
    }

    //===============================================================================================
    //===============================================================================================

    // Follow the choreography specified in the robot action file.
    public void actionLoop(List<RobotXMLElement> pActions) throws InterruptedException, XPathException, IOException {
            for (RobotXMLElement action : pActions) {
                executeAction(action); // no, but doCommand may change that
            }
    }

    // Using the XML elements and attributes from the configuration file RobotAction.xml,
    // execute the action.
    public void executeAction(RobotXMLElement pAction) throws InterruptedException, XPathException, IOException {

        // Set up XPath access to the current action command.
        XPathAccess commandXPath = new XPathAccess(pAction);
        String commandName = pAction.getRobotXMLElementName().toUpperCase();
        RobotLogCommon.d(TAG, "Executing robot action " + commandName);

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

            case "SHOOT": {
                double shootVelocity = commandXPath.getDouble("shootVelocity");
                double intakeVelocity = commandXPath.getDouble("intakeVelocity");
                int waitTime = commandXPath.getInt("waitTime");
                int dip = commandXPath.getInt("shootVelocityDip");

                //optional parameters
                int maxShotCount = commandXPath.getInt("maxShotCount", 3);
                boolean powerShot = commandXPath.getBoolean("powerShot", false);
                int shotWaitTime = commandXPath.getInt("shotWaitTime", 500);
                int beginningWaitTime = commandXPath.getInt("beginningWaitTime", 500);

                robot.shooter.shootMotor.setVelocity(shootVelocity);
                double currentVelocity = robot.shooter.shootMotor.getVelocity();
                while (currentVelocity < shootVelocity) {
                    currentVelocity = robot.shooter.shootMotor.getVelocity();
                    sleep(20);
                }

                sleep(beginningWaitTime);

                int shotCount = 0;
                linearOpMode.telemetry.addData("Shots ", shotCount);
                linearOpMode.telemetry.update();
                long timeout = System.currentTimeMillis() + waitTime;
                while (System.currentTimeMillis() < timeout) {
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

                            break;
                        }
                        sleep(1);


                    }
                    robot.shooter.triggerServo.setState("rest");

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
                robot.shooter.intakeMotor.setVelocity(0);
                break;
            }

            case "ELEVATOR_UP": {
                robot.shooter.moveElevatorUp();

                break;
            }

            case "ELEVATOR_DOWN": {
                robot.shooter.moveElevatorDown();

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

