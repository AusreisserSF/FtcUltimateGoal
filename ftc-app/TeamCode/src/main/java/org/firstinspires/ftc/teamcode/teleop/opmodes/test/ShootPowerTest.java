package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Test")
//@Disabled
public class ShootPowerTest extends TeleOpBase {
    private final double IncrementVelocity = 0;
    private final Button velocityIncrementButton = new Button();
    private final Button velocityDecrementButton = new Button();
    private double shootVelocity = 0;

    @Override
    protected void initialize() {
        telemetry.addData("shoot", "right stick y");
        telemetry.update();
    }

    @Override
    protected void update() {

        velocityIncrementButton.update(gamepad1.y);
        velocityDecrementButton.update(gamepad1.a);

        robot.ringShooter.intakeMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        robot.ringShooter.liftMotor.setPower(gamepad1.left_stick_y);
        robot.ringShooter.shootMotor.setVelocity(gamepad1.right_stick_y * shootVelocity);

        if (velocityIncrementButton.is(Button.State.DOWN)) {
            shootVelocity += IncrementVelocity;
        } else if (velocityDecrementButton.is(Button.State.DOWN)) {
            shootVelocity -= IncrementVelocity;
        }

        telemetry.addData("shoot velocity", shootVelocity);
        telemetry.addData("actual shoot velocity", robot.ringShooter.shootMotor.getVelocity());

        telemetry.update();

    }

}
