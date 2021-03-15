package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.LCHSMath;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Test")
//@Disabled
public class WobbleServoTest extends TeleOpBase {

    private final Button servoIncrementButton = new Button();
    private final Button servoDecrementButton = new Button();
    private double servoPosition = 0;

    @Override
    protected void initialize() {
        servoPosition = LCHSMath.round(robot.wobbleArm.servo.getPosition(), 1);
        updateTelemetry();
    }

    @Override
    protected void update() {
        updateButtons();
        updateTelemetry();
        robot.wobbleArm.flipMotor.setPower(gamepad1.right_stick_x);
        if (servoIncrementButton.is(Button.State.DOWN)) {
            servoPosition += 0.1;
            robot.wobbleArm.servo.setPosition(servoPosition);
        } else if (servoDecrementButton.is(Button.State.DOWN)) {
            servoPosition -= 0.1;
            robot.wobbleArm.servo.setPosition(servoPosition);
        }
    }

    private void updateButtons() {
        servoIncrementButton.update(gamepad1.y);
        servoDecrementButton.update(gamepad1.a);
    }

    private void updateTelemetry() {
        telemetry.addData("servo position", servoPosition);
        telemetry.addData("-----", "controls");
        telemetry.addData("flip motor", "right joystick x");
        telemetry.addData("increment servo", "y");
        telemetry.addData("decrement servo", "a");
        telemetry.update();
    }
}
