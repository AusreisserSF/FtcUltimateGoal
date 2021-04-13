package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.firstinspires.ftc.ftcdevcommon.android.WorkingDirectory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.xml.RobotConfigXML;
import org.firstinspires.ftc.teamcode.hardware.imu.OptimizedIMU;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.List;

import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPathExpressionException;

public class LCHSRobot {

    private static LCHSRobot instance;

    public static LCHSRobot newInstance(LinearOpMode opMode) {
        instance = new LCHSRobot(opMode);
        return instance;
    }

    public static LCHSRobot getInstance() {
        return instance;
    }

    public final LinearOpMode opMode;
    public final HardwareMap hardwareMap;
    public WebcamName webcam1Name;
    public OptimizedIMU imu;

    public RobotConfigXML configXML;

    public DriveTrain driveTrain;
    public WobbleArm wobbleArm;
    public RingShooter shooter;

    private static final String TAG = "LCHSRobot";

    public LCHSRobot(LinearOpMode opMode) {

        String workingDirectory = WorkingDirectory.getWorkingDirectory();
        String xmlDirectory = workingDirectory + RobotConstants.xmlDir;

        try {
            configXML = new RobotConfigXML(xmlDirectory);

            this.opMode = opMode;
            this.hardwareMap = opMode.hardwareMap;

            // See if configXML contains an entry for a webcam
            XPathAccess configXPath;
            configXPath = configXML.getPath("WEBCAM");
            String webcamInConfiguration = configXPath.getString("present", "no");
            if (webcamInConfiguration.equals("yes"))
                webcam1Name = this.hardwareMap.get(WebcamName.class, "Webcam 1");
            // if not, leave webcam1name null and test in FTCAuto

            driveTrain = new DriveTrain(hardwareMap);
            wobbleArm = new WobbleArm(hardwareMap, configXML);
            shooter = new RingShooter(hardwareMap, configXML);

        } catch (ParserConfigurationException pex) {
            throw new AutonomousRobotException(TAG, "DOM parser Exception " + pex.getMessage());
        } catch (SAXException sx) {
            throw new AutonomousRobotException(TAG, "SAX Exception " + sx.getMessage());
        } catch (XPathExpressionException xpex) {
            throw new AutonomousRobotException(TAG, "Xpath Expression Exception " + xpex.getMessage());
        }
        catch (IOException iex) {
            throw new AutonomousRobotException(TAG, "IOException " + iex.getMessage());
        }

    }

    public void initializeIMU() {
        imu = new OptimizedIMU(hardwareMap, opMode);
    }

    public double getBatteryVoltage() {
        List<VoltageSensor> voltageSensors = hardwareMap.getAll(VoltageSensor.class);
        double voltage = 0;
        for (VoltageSensor sensor : voltageSensors) {
            voltage = Math.max(sensor.getVoltage(), voltage);
        }
        return voltage;
    }
}
