package org.firstinspires.ftc.teamcode.teleop.opmodes.drive;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.math.Pose;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;

@TeleOp(group="Drive")
//@Disabled
public class PracticeDrive extends BaseDrive {

    @Override
    protected void update() {
        if (gamepad1.left_bumper) {
            drivePowerFactor = 0.5;
        } else if (gamepad1.right_bumper) {
            drivePowerFactor = 1.0;
        } else {
            drivePowerFactor = 0.75;
        }
        telemetry.addData("power", drivePowerFactor);
        telemetry.update();
        updateDrive();
    }


}