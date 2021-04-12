package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Test")
//@Disabled
public class ShootPowerTest extends TeleOpBase {
    private final double IncrementVelocity = 100;
    private final Button velocityIncrementButton = new Button();
    private final Button velocityDecrementButton = new Button();
    private double shootVelocity = 1500;

    @Override
    protected void initialize() {
        telemetry.addData("shoot", "right stick y");
        telemetry.update();
    }

    @Override
    protected void update() {

        velocityIncrementButton.update(gamepad1.y);
        velocityDecrementButton.update(gamepad1.a);

        robot.shooter.elevatorMotor.setPower(gamepad1.left_stick_y + gamepad1.right_stick_y * 0.2);
        robot.shooter.shootMotor.setVelocity(gamepad1.right_stick_y * shootVelocity);

        if (velocityIncrementButton.is(Button.State.TAP)) {
            shootVelocity += IncrementVelocity;
        } else if (velocityDecrementButton.is(Button.State.TAP)) {
            shootVelocity -= IncrementVelocity;
        }

        telemetry.addData("shoot velocity", shootVelocity);
        telemetry.addData("actual shoot velocity", robot.shooter.shootMotor.getVelocity());

        telemetry.update();

    }

}
