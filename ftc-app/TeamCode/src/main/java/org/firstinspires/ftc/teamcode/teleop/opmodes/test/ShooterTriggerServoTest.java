package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.LCHSMath;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Test")
//@Disabled
public class ShooterTriggerServoTest extends TeleOpBase {

    private final Button servoIncrementButton = new Button();
    private final Button servoDecrementButton = new Button();
    private double servoPosition = 0;

    @Override
    protected void initialize() {
        servoPosition = LCHSMath.round(robot.shooter.triggerServo.getPosition(), 1);
        updateTelemetry();
    }

    @Override
    protected void update() {
        updateButtons();
        updateTelemetry();
        if (servoIncrementButton.is(Button.State.TAP)) {
            servoPosition += 0.025;
            robot.shooter.triggerServo.setPosition(servoPosition);
        } else if (servoDecrementButton.is(Button.State.TAP)) {
            servoPosition -= 0.025;
            robot.shooter.triggerServo.setPosition(servoPosition);
        }
    }

    private void updateButtons() {
        servoIncrementButton.update(gamepad1.y);
        servoDecrementButton.update(gamepad1.a);
    }

    private void updateTelemetry() {
        telemetry.addData("servo position", servoPosition);
        telemetry.addData("-----", "controls");
        telemetry.addData("increment servo", "y");
        telemetry.addData("decrement servo", "a");
        telemetry.update();
    }
}
