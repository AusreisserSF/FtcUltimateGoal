package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Drive")
//@Disabled
public class BaseDrive extends TeleOpBase {

    protected double drivePowerFactor = 1.0;
    protected Pose drivePose = new Pose();

    @Override
    protected void initialize() { }

    @Override
    protected void update() {
        updateDrive();
    }

    protected void updateDrive() {
        // up on the gamepad stick is negative
        drivePose.x = Math.pow(gamepad1.left_stick_x, 3);
        drivePose.y = -Math.pow(gamepad1.left_stick_y, 3);
        drivePose.r = -Math.pow(gamepad1.right_stick_x, 3);
        robot.driveTrain.drive(drivePose, drivePowerFactor);
    }

}