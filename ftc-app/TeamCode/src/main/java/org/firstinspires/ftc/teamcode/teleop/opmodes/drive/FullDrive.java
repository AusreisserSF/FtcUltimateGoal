package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.WobbleArm;
import org.firstinspires.ftc.teamcode.robot.WobbleArm.ServoState;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

@TeleOp(group="Drive")
//@Disabled
public class FullDrive extends BaseDrive {

    private final Button wobbleServoButton = new Button();

    @Override
    protected void update() {
        updatePlayerOne();
        updatePlayerTwo();
        updateTelemetry();
    }

    private void updatePlayerOne() {
        if (gamepad1.left_bumper) {
            drivePowerFactor = 0.5;
        } else if (gamepad1.right_bumper) {
            drivePowerFactor = 1.0;
        } else {
            drivePowerFactor = 0.75;
        }
        updateDrive();
    }

    private void updatePlayerTwo() {
        wobbleServoButton.update(gamepad2.b);

        robot.ringShooter.intakeMotor.setPower(gamepad2.left_stick_y + gamepad2.right_stick_y * 0.2);
        robot.ringShooter.liftMotor.setPower(gamepad2.left_stick_y);
        robot.ringShooter.shootMotor.setPower(gamepad2.right_stick_y);

        double wobbleFlipPower = (gamepad2.dpad_up ? 1 : 0) - (gamepad2.dpad_down ? 1 : 0);
        robot.wobbleArm.flipMotor.setPower(wobbleFlipPower * WobbleArm.FLIP_POWER_FACTOR);

        if (wobbleServoButton.is(Button.State.DOWN)) {
            ServoState servoState = robot.wobbleArm.getServoState() == ServoState.HOLD ? ServoState.RELEASE : ServoState.HOLD;
            robot.wobbleArm.setServoState(servoState);
        }
    }

    private void updateTelemetry() {
        telemetry.addData("power", drivePowerFactor);
        telemetry.update();
    }

}