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
    private final Button ringShooterButton = new Button();
    private final Button ringPowerShotButton = new Button();


    @Override
    protected void update() {
        updatePlayerOne();
        updatePlayerTwo();
        updateTelemetry();
    }

    private void updatePlayerOne() {
        wobbleServoButton.update(gamepad1.b);


        if (gamepad1.left_bumper) {
            drivePowerFactor = 0.5;
        } else if (gamepad1.right_bumper) {
            drivePowerFactor = 1.0;
        } else {
            drivePowerFactor = 0.75;
        }

        if (wobbleServoButton.is(Button.State.TAP)) {
            // boolean ? value if true : value if false
            ServoState servoState = robot.wobbleArm.getServoState() == WobbleArm.ServoState.HOLD ? WobbleArm.ServoState.RELEASE : WobbleArm.ServoState.HOLD;
            robot.wobbleArm.setServoState(servoState);
        }

        updateDrive();
    }

    private void updatePlayerTwo() {
        gateServoButton.update(gamepad2.y);
        ringShooterButton.update(gamepad2.x);
        ringPowerShotButton.update(gamepad2.a);

        robot.ringShooter.intakeMotor.setPower(gamepad2.left_stick_y + gamepad2.right_stick_y * 0.2);
        robot.ringShooter.liftMotor.setPower(gamepad2.left_stick_y);
        robot.ringShooter.shootMotor.setPower(gamepad2.right_stick_y);

        //Add an if/else if function for the two buttons
        //If for X, Else if for A, Else for none

        robot.ringShooter.shootMotor.setVelocity(   ringShooterButton.is(Button.State.HELD) ? 1800 : 0  );

        //robot.ringShooter.shootMotor.setVelocity(  ringPowerShotButton.is(Button.State.HELD) ? 1500 : 0  );

        double wobbleFlipPower = (gamepad2.dpad_up ? 1 : 0) - (gamepad2.dpad_down ? 1 : 0);
        robot.wobbleArm.flipMotor.setPower(wobbleFlipPower * WobbleArm.FLIP_POWER_FACTOR);

        if (gateServoButton.is(Button.State.TAP)) {
            RingShooter.ServoState servoState = robot.ringShooter.getServoState() == RingShooter.ServoState.UP ? RingShooter.ServoState.DOWN : RingShooter.ServoState.UP;
            robot.ringShooter.setServoState(servoState);
        }

    }

    private void updateTelemetry() {
        telemetry.addData("drive power", drivePowerFactor);
        telemetry.addData("shoot velocity", robot.ringShooter.shootMotor.getVelocity());
        telemetry.update();
    }

}