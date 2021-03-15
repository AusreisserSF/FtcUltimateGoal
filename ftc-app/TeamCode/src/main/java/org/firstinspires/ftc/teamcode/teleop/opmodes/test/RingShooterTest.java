package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.LCHSMath;
import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Test")
@Disabled
public class RingShooterTest extends TeleOpBase {

    @Override
    protected void initialize() {
        telemetry.addData("intake", "right and left triggers");
        telemetry.addData("lift", "left stick y");
        telemetry.addData("shoot", "right stick y");
        telemetry.update();
    }

    @Override
    protected void update() {
        robot.ringShooter.intakeMotor.setPower(gamepad1.right_trigger - gamepad1.left_trigger);
        robot.ringShooter.liftMotor.setPower(gamepad1.left_stick_y);
        robot.ringShooter.shootMotor.setPower(gamepad1.right_stick_y);
    }

}
