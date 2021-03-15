package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.math.LCHSMath;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Test")
//@Disabled
public class WobbleMotorTest extends TeleOpBase {

    private final Button incrementButton = new Button();
    private final Button decrementButton = new Button();
    private final int incrementValue = 100;
    private int position = 0;

    @Override
    protected void initialize() {
        position = robot.wobbleArm.flipMotor.getCurrentPosition();
        robot.wobbleArm.flipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        updateTelemetry();
    }

    @Override
    protected void update() {
        updateButtons();
        updateTelemetry();
        if (incrementButton.is(Button.State.DOWN)) {
            position += incrementValue;
            robot.wobbleArm.flipMotor.setTargetPosition(position);
        } else if (decrementButton.is(Button.State.DOWN)) {
            position -= incrementValue;
            robot.wobbleArm.flipMotor.setTargetPosition(position);
        }
    }

    private void updateButtons() {
        incrementButton.update(gamepad1.y);
        decrementButton.update(gamepad1.a);
    }

    private void updateTelemetry() {
        telemetry.addData("motor position", position);
        telemetry.addData("-----", "controls");
        telemetry.addData("increment position", "y");
        telemetry.addData("decrement position", "a");
        telemetry.update();
    }
}
