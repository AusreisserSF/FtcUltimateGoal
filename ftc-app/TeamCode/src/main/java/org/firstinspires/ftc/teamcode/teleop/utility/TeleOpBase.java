package org.firstinspires.ftc.teamcode.teleop.utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robot.LCHSRobot;
import org.xml.sax.SAXException;

import java.io.IOException;

import javax.xml.parsers.ParserConfigurationException;

public abstract class TeleOpBase extends LinearOpMode {

    protected LCHSRobot robot;

    protected abstract void initialize();
    protected abstract void update();

    @Override
    public void runOpMode() {
        telemetry.addData("Initializing...", "Please wait until complete");
        telemetry.update();

        try {
            robot = LCHSRobot.newInstance(this);
        }
        catch (ParserConfigurationException | IOException | SAXException ex) {
            //**TODO this is against the law!!
        }
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
