package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.RobotConstants;

import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.RobotConstantsUltimateGoal;

@Autonomous(name = "RedInside", group = "TeamCode")
//@Disabled
public class RedInside extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        LCHSAutoDispatch dispatch = new LCHSAutoDispatch();
        dispatch.runOpMode(RobotConstantsUltimateGoal.OpMode.RED_INSIDE, RobotConstants.Alliance.RED, this);
    }
}
