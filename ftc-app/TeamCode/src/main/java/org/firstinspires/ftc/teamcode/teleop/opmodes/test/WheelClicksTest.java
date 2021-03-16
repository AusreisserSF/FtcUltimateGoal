package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teleop.opmodes.drive.BaseDrive;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;

@TeleOp(group="Test")
@Disabled
public class WheelClicksTest extends BaseDrive {

    private final Button resetButton = new Button();

    @Override
    protected void initialize() {
        robot.driveTrain.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        robot.driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        updateTelemetry();
    }

    @Override
    protected void update() {
        updateResetButton();
        updateTelemetry();
        updateDrive();
    }

    private void updateResetButton() {
        resetButton.update(gamepad1.a);
        if (resetButton.is(Button.State.TAP)) {
            robot.driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }

    private void updateTelemetry() {
        double lf = robot.driveTrain.lf.getCurrentPosition();
        double rf = robot.driveTrain.rf.getCurrentPosition();
        double lb = robot.driveTrain.lb.getCurrentPosition();
        double rb = robot.driveTrain.rb.getCurrentPosition();
        telemetry.addData("lf", lf);
        telemetry.addData("rf", rf);
        telemetry.addData("lb", lb);
        telemetry.addData("rb", rb);
        telemetry.addData("avg", (lf + rf + lb + rb) / 4.0 );
        telemetry.addData("------", "controls");
        telemetry.addData("reset clicks", "a");
        telemetry.update();
    }

}
