package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Test")
//@Disabled
public class ShootPowerTest extends TeleOpBase {

    private final Button powerIncrementButton = new Button();
    private final Button powerDecrementButton = new Button();
    private double shootPower = 0;

    @Override
    protected void initialize() {
        telemetry.addData("shoot", "right stick y");
        telemetry.update();
    }

    @Override
    protected void update() {

        powerIncrementButton.update(gamepad1.y);
        powerDecrementButton.update(gamepad1.a);

        robot.ringShooter.intakeMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        robot.ringShooter.liftMotor.setPower(gamepad1.left_stick_y);
        robot.ringShooter.shootMotor.setPower(gamepad1.right_stick_y * shootPower);

        if (powerIncrementButton.is(Button.State.DOWN)) {
            shootPower += 0.1;
        } else if (powerDecrementButton.is(Button.State.DOWN)) {
            shootPower -= 0.1;
        }

        telemetry.addData("shoot power", shootPower);
        telemetry.update();

    }

}
