package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.RobotConstantsUltimateGoal;

@Autonomous(name = "Test", group = "TeamCode")
//@Disabled
public class Test extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        LCHSAutoDispatch dispatch = new LCHSAutoDispatch();
        dispatch.runOpMode(RobotConstantsUltimateGoal.OpMode.TEST, RobotConstants.Alliance.RED, this);
    }
}


