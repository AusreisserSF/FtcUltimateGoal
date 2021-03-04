package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Test")
@Disabled
public class WheelDirectionTest extends TeleOpBase {
    @Override
    protected void initialize() {
        telemetry.addData("lf", "y");
        telemetry.addData("rf", "b");
        telemetry.addData("lb", "x");
        telemetry.addData("rb", "a");
        telemetry.update();
    }

    @Override
    protected void update() {
        robot.driveTrain.lf.setPower(( gamepad1.y ? 1 : 0 ));
        robot.driveTrain.rf.setPower(( gamepad1.b ? 1 : 0 ));
        robot.driveTrain.lb.setPower(( gamepad1.x ? 1 : 0 ));
        robot.driveTrain.rb.setPower(( gamepad1.a ? 1 : 0 ));
    }
}
