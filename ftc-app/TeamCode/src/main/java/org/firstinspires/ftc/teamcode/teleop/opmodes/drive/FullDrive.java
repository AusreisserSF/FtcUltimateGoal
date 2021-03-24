package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.RingShooter;
import org.firstinspires.ftc.teamcode.robot.WobbleArm;
import org.firstinspires.ftc.teamcode.robot.WobbleArm.ServoState;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

@TeleOp(group="Drive")
//@Disabled
public class FullDrive extends BaseDrive {

    private final Button wobbleServoButton = new Button();
    private final Button gateServoButton = new Button();
    private final Button wobbleRestPosition = new Button();
    private final Button ringPowerShotButton = new Button();
    private final Button wobbleFlipButton = new Button();
    private final Button highGoalButton = new Button();

    private final double shootVelocity = 2000;
    private final double intakeVelocity = 2000;
    private final double lifterVelocity = 2000;

    private boolean isShootingHighGoal = false;


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
    }

    private void updatePlayerTwo() {
        updatePowerShot();
        updateGateServo();
        updateWobbleFlip();
        updateIntake();
        updateShootHighGoal();
    }

    private void updateButtons() {
        wobbleServoButton.update(gamepad1.b);
        wobbleRestPosition.update(gamepad1.x);

        gateServoButton.update(gamepad2.y);
        wobbleFlipButton.update(gamepad2.b);
        ringPowerShotButton.update(gamepad2.a);
        highGoalButton.update(gamepad2.x);
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

    private void updateWobbleServo() {
        if (wobbleServoButton.is(Button.State.TAP)) {
            ServoState servoState = robot.wobbleArm.getServoState() == WobbleArm.ServoState.HOLD ? WobbleArm.ServoState.RELEASE : WobbleArm.ServoState.HOLD;
            robot.wobbleArm.setServoState(servoState);
        }
        if (wobbleRestPosition.is(Button.State.TAP)){
            robot.wobbleArm.setServoState(robot.wobbleArm.getServoState() == WobbleArm.ServoState.REST ? WobbleArm.ServoState.HOLD : WobbleArm.ServoState.REST);
        }
    }

    private void updatePowerShot() {
        robot.ringShooter.shootMotor.setPower(gamepad2.right_stick_y);

        if (ringPowerShotButton.is(Button.State.TAP)) {
            robot.ringShooter.shootMotor.setVelocity(0);
            if (gamepad2.left_bumper) {
                robot.ringShooter.shootMotor.setVelocity(1600);
            } else if (gamepad2.right_bumper) {
                robot.ringShooter.shootMotor.setVelocity(2000); //Allows for Rapid Fire
            }
        }
    }

    private void updateShootHighGoal() {

        if (highGoalButton.is(Button.State.TAP)) {
            isShootingHighGoal = !isShootingHighGoal;

            if (isShootingHighGoal) {
                robot.ringShooter.shootMotor.setVelocity(shootVelocity);
            } else {
                robot.ringShooter.shootMotor.setVelocity(0);
                robot.ringShooter.intakeMotor.setVelocity(0);
                robot.ringShooter.liftMotor.setVelocity(0);
            }
        }

        if (isShootingHighGoal) {
            double currentVelocity = robot.ringShooter.shootMotor.getVelocity();

            if (currentVelocity < shootVelocity) {
                robot.ringShooter.intakeMotor.setVelocity(0);
                robot.ringShooter.liftMotor.setVelocity(0);
            } else {
                robot.ringShooter.intakeMotor.setVelocity(intakeVelocity);
                robot.ringShooter.liftMotor.setVelocity(lifterVelocity);
            }
        }

    }


    private void updateGateServo() {
        if (gateServoButton.is(Button.State.TAP)) {
            RingShooter.ServoState servoState = robot.ringShooter.getServoState() == RingShooter.ServoState.UP ? RingShooter.ServoState.DOWN : RingShooter.ServoState.UP;
            robot.ringShooter.setServoState(servoState);
        }
    }

    private void updateIntake() {
        double intakeVelocity = (gamepad2.dpad_up ? 2000 : 0) - (gamepad2.dpad_down ? 2000 : 0);
        robot.ringShooter.intakeMotor.setVelocity(intakeVelocity);
        robot.ringShooter.liftMotor.setVelocity(intakeVelocity);

    }

    private void updateWobbleFlip() {
        if (wobbleFlipButton.is(Button.State.TAP)) {
            robot.wobbleArm.setFlipState(robot.wobbleArm.getFlipState() == WobbleArm.FlipState.IN ? WobbleArm.FlipState.INTAKE : WobbleArm.FlipState.IN);
        }
    }

    private void updateTelemetry() {
        telemetry.addData("drive power", drivePowerFactor);
        telemetry.addData("shoot velocity", robot.ringShooter.shootMotor.getVelocity());
        telemetry.addData("intake velocity", robot.ringShooter.intakeMotor.getVelocity());
        telemetry.update();
    }



}