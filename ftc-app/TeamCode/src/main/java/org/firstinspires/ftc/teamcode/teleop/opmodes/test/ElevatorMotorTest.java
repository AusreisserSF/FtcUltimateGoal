package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.robot.RingShooter;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Test")
//@Disabled
public class    ElevatorMotorTest extends TeleOpBase {

    private final Button incrementButton = new Button();
    private final Button decrementButton = new Button();
    private final int incrementValue = 10;
    private int position = 0;
    private boolean isGoingToPosition = false;

    @Override
    protected void initialize() {
        position = robot.shooter.elevatorMotor.getCurrentPosition();
        robot.shooter.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        updateTelemetry();
    }

    @Override
    protected void update() {
        updateButtons();
        updateTelemetry();
        if (incrementButton.is(Button.State.TAP)) {
            position += incrementValue;
            runElevatorMotorToPosition();
        } else if (decrementButton.is(Button.State.TAP)) {
            position -= incrementValue;
            runElevatorMotorToPosition();
        } else if (!isGoingToPosition || (gamepad1.dpad_up || gamepad1.dpad_down)) {
            isGoingToPosition = false;
            robot.shooter.elevatorMotor.checkAndSetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            double elevatorPower = (gamepad1.dpad_up ? 1 : 0) - (gamepad1.dpad_down ? 1 : 0);
            robot.shooter.elevatorMotor.setPower(elevatorPower * 0.5);
        }
    }

    private void updateButtons() {
        incrementButton.update(gamepad1.y);
        decrementButton.update(gamepad1.a);
    }

    private void updateTelemetry() {
        telemetry.addData("target position", robot.shooter.elevatorMotor.getTargetPosition());
        telemetry.addData("actual position", robot.shooter.elevatorMotor.getCurrentPosition());
        telemetry.addData("-----", "controls");
        telemetry.addData("increment position", "y");
        telemetry.addData("decrement position", "a");
        telemetry.update();
    }

    private void runElevatorMotorToPosition() {
        isGoingToPosition = true;
        robot.shooter.moveElevatorUp();
    }
}


