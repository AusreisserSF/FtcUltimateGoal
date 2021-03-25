package org.firstinspires.ftc.teamcode.teleop.utility;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.teamcode.robot.LCHSRobot;
import org.xml.sax.SAXException;

import java.io.IOException;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathExpressionException;

public abstract class TeleOpBase extends LinearOpMode {

    private static String TAG = "TeleOpBase";
    protected LCHSRobot robot;

    protected abstract void initialize();

    protected abstract void update();

    @Override
    public void runOpMode() {
        telemetry.addData("Initializing...", "Please wait until complete");
        telemetry.update();

        try {
            robot = LCHSRobot.newInstance(this);
        } catch (ParserConfigurationException pex) {
            throw new AutonomousRobotException(TAG, "DOM parser Exception " + pex.getMessage());
        } catch (SAXException sx) {
            throw new AutonomousRobotException(TAG, "SAX Exception " + sx.getMessage());
        } catch (IOException iex) {
            throw new AutonomousRobotException(TAG, "IOException " + iex.getMessage());
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
