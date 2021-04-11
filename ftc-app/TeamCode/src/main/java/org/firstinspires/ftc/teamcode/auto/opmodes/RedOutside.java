package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.RobotConstants;

import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.RobotConstantsUltimateGoal;

@Autonomous(name = "RedOutside", group = "TeamCode")
//@Disabled
public class RedOutside extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        LCHSAutoDispatch dispatch = new LCHSAutoDispatch();
        dispatch.runOpMode(RobotConstantsUltimateGoal.OpMode.RED_OUTSIDE, RobotConstants.Alliance.RED, this);
    }
}