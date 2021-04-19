package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.firstinspires.ftc.ftcdevcommon.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.auto.RobotActionCommon;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.RobotConstantsUltimateGoal;
import org.firstinspires.ftc.teamcode.auto.xml.RobotActionXML;
import org.firstinspires.ftc.teamcode.robot.WobbleArm;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.xml.sax.SAXException;

import java.io.IOException;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathException;
import javax.xml.xpath.XPathExpressionException;

@TeleOp(group="Drive")
//@Disabled
public class FullDrive extends BaseDrive {

    private final Button wobbleServoButton = new Button();
    private final Button wobbleRestPosition = new Button();
    private final Button ringPowerShotButton = new Button();
    private final Button wobbleFlipButton = new Button();
    private final Button highGoalButton = new Button();
    private final Button elevatorButton = new Button();
    private final Button flickerServo = new Button();
    private final Button powerShotMoveButton = new Button();

    private double highGoalShootVelocity;
    private double powershotHighShootVelocity;
    private double powershotLowShootVelocity;
    private double intakeVelocity;

    private boolean isShootingHighGoal = false;

    // For reading RobotAction.xml in TeleOp
    private RobotActionXML actionXML;
    private RobotActionCommon commonActions;

    @Override
    protected void initialize() {

        try {
            //**TODO Where in TeleOp can I close the log?
            RobotLogCommon.initialize(WorkingDirectory.getWorkingDirectory() + RobotConstants.logDir);
            String xmlDirectory = WorkingDirectory.getWorkingDirectory() + RobotConstants.xmlDir;
            actionXML = new RobotActionXML(xmlDirectory);
            commonActions = new RobotActionCommon(robot, this);
        }
        catch (IOException | SAXException | ParserConfigurationException initEx) {
            throw new AutonomousRobotException("FullDrive", "XML error in initialize");
        }

        XPathAccess config = robot.configXML.getPath("FULL_DRIVE");
        try {
            highGoalShootVelocity = config.getDouble("high_goal", 0);
            powershotHighShootVelocity = config.getDouble("powershot_high", 0);
            powershotLowShootVelocity = config.getDouble("powershot_low", 0);
            intakeVelocity = config.getDouble("intake", 0);
        } catch (XPathExpressionException e) {
            throw new AutonomousRobotException("FullDrive", "XPath error in initialize");
        }

        // Required for executing RobotAction.xml commands in TeleOp.
        // in both Autonomous and TeleOp
        robot.initializeIMU();
    }

    @Override
    protected void update() {
        updateButtons();
        updatePlayerOne();
        updatePlayerTwo();
        updateTelemetry();
    }

    private void updatePlayerOne() {
        updateDrivePower();
        updateDrive();
        updateWobbleServo();
        updatePowerShotMove();
    }

    private void updatePlayerTwo() {
        updatePowerShot();
        updateShootHighGoal();
        updateWobbleFlip();
        updateIntake();
        updateElevator();
        updateFlicker();
    }

    private void updateButtons() {
        wobbleServoButton.update(gamepad1.b);
        wobbleRestPosition.update(gamepad1.x);
        powerShotMoveButton.update(gamepad1.y);

        wobbleFlipButton.update(gamepad2.b);
        ringPowerShotButton.update(gamepad2.a);
        elevatorButton.update(gamepad2.y);
        flickerServo.update(gamepad2.x);
    }

    private void updateDrivePower() {
        if (gamepad1.left_bumper) {
            drivePowerFactor = 0.5;
        } else if (gamepad1.right_bumper) {
            drivePowerFactor = 0.75;
        } else {
            drivePowerFactor = 1.0;
        }
    }

    private void updatePowerShotMove() {

        if (powerShotMoveButton.is(Button.State.TAP)){
            try {
                RobotActionXML.RobotActionData actionData = actionXML.getOpModeData(RobotConstantsUltimateGoal.OpMode.TELEOP_POWER_SHOT.toString());
                commonActions.actionLoop(actionData.actions);
            }
            catch (XPathExpressionException xpEx) {
                throw new AutonomousRobotException("FullDrive", "XPath error in updatePowerShotMove");
            } catch (InterruptedException e) {
                RobotLogCommon.e("FullDrive", "InterruptedException in updatePowerShotMove");
            } catch (IOException e) {
                throw new AutonomousRobotException("FullDrive", "IOException in updatePowerShotMove");
            } catch (XPathException e) {
                throw new AutonomousRobotException("FullDrive", "XPathException in updatePowerShotMove");
            }
        }
    }

    private void updateWobbleServo() {
        if (wobbleServoButton.is(Button.State.TAP)) {
            String state = robot.wobbleArm.servo.getState().equals("hold") ? "release" : "hold";
            robot.wobbleArm.servo.setState(state);
        }
        if (wobbleRestPosition.is(Button.State.TAP)){
            String state = robot.wobbleArm.servo.getState().equals("rest") ? "hold" : "rest";
            robot.wobbleArm.servo.setState(state);
        }
    }

    private void updatePowerShot() {
        robot.shooter.shootMotor.setPower(gamepad2.right_stick_y);

        if (ringPowerShotButton.is(Button.State.TAP)) {
            robot.shooter.shootMotor.setVelocity(0);
            if (gamepad2.left_bumper) {
                robot.shooter.shootMotor.setVelocity(powershotLowShootVelocity);

            } else if (gamepad2.right_bumper) {
                robot.shooter.shootMotor.setVelocity(powershotHighShootVelocity); // Allows for Rapid Fire
            }
        }
    }

    private void updateShootHighGoal() {

        if (highGoalButton.is(Button.State.TAP)) {
            isShootingHighGoal = !isShootingHighGoal;

            if (isShootingHighGoal) {
                robot.shooter.shootMotor.setVelocity(highGoalShootVelocity);
            } else {
                robot.shooter.shootMotor.setVelocity(0);
                robot.shooter.intakeMotor.setVelocity(0);
            }
        }

        /*
        if (isShootingHighGoal) {
            double currentVelocity = robot.shooter.shootMotor.getVelocity();

            if (currentVelocity < highGoalShootVelocity) {
                robot.shooter.intakeMotor.setVelocity(0);
            } else {
                robot.shooter.intakeMotor.setVelocity(intakeVelocity);
            }
        }
        */

    }

    private void updateFlicker(){

        if (flickerServo.is(Button.State.TAP)){
            robot.shooter.triggerServo.setState("out");
            sleep(75);
            robot.shooter.triggerServo.setState("rest");

            if (gamepad2.dpad_left){
                robot.shooter.triggerServo.setState("extend");
                sleep(100);
                robot.shooter.triggerServo.setState("rest");
            }
        }
    }

    private void updateIntake() {
        double power = (gamepad2.dpad_up ? 1 : 0) - (gamepad2.dpad_down ? 1 : 0);
        robot.shooter.intakeMotor.setVelocity(power * intakeVelocity);
    }

    private void updateWobbleFlip() {
        if (wobbleFlipButton.is(Button.State.TAP)) {
            robot.wobbleArm.setFlipState(robot.wobbleArm.getFlipState() == WobbleArm.FlipState.IN ? WobbleArm.FlipState.INTAKE : WobbleArm.FlipState.IN);
        }
    }

    private void updateElevator() {
        if (elevatorButton.is(Button.State.TAP)) {
            robot.shooter.toggleElevator();
        }
    }

    private void updateTelemetry() {
        telemetry.addData("drive power", drivePowerFactor);
        telemetry.addData("shoot velocity", robot.shooter.shootMotor.getVelocity());
        telemetry.addData("intake velocity", robot.shooter.intakeMotor.getVelocity());
        telemetry.update();
    }

}