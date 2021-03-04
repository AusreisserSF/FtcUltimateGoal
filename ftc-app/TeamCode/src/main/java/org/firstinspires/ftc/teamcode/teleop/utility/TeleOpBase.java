package org.firstinspires.ftc.teamcode.teleop.utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.LCHSRobot;

public abstract class TeleOpBase extends LinearOpMode {

    protected LCHSRobot robot;

    protected abstract void initialize();
    protected abstract void update();

    @Override
    public void runOpMode() {
        telemetry.addData("Initializing...", "Please wait until complete");
        telemetry.update();

        robot = LCHSRobot.newInstance(this);
        initialize();

        telemetry.addData("Initialized!", "Ready to run");
        telemetry.update();

        waitForStart();
        while (opModeIsActive()) {
            update();
        }
        onStop();
    }

    protected void onStop() {
        robot.driveTrain.stop();
    }

}
