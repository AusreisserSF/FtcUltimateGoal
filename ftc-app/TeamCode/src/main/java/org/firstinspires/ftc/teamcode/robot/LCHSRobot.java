package org.firstinspires.ftc.teamcode.robot;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.android.WorkingDirectory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.xml.RobotConfigXML;
import org.firstinspires.ftc.teamcode.hardware.imu.OptimizedIMU;
import org.xml.sax.SAXException;

import java.io.IOException;
import java.util.List;

import javax.xml.parsers.ParserConfigurationException;

public class LCHSRobot {

    private static LCHSRobot instance;

    public static LCHSRobot newInstance(LinearOpMode opMode) throws AutonomousRobotException {
        instance = new LCHSRobot(opMode);
        return instance;
    }

    public static LCHSRobot getInstance() {
        return instance;
    }

    public final LinearOpMode opMode;
    public final HardwareMap hardwareMap;
    public WebcamName webcam1Name;
    public T265Camera slamra;
    public OptimizedIMU imu;

    public RobotConfigXML configXML;

    public DriveTrain driveTrain;
    public WobbleArm wobbleArm;
    public RingShooter ringShooter;

    private static final String TAG = "LCHSRobot";


    public LCHSRobot(LinearOpMode opMode) throws AutonomousRobotException {

        String workingDirectory = WorkingDirectory.getWorkingDirectory();
        String xmlDirectory = workingDirectory + RobotConstants.xmlDir;

        try {
            configXML = new RobotConfigXML(xmlDirectory);
        } catch (ParserConfigurationException pex) {
            throw new AutonomousRobotException(TAG, "DOM parser Exception " + pex.getMessage());
        } catch (SAXException sx) {
            throw new AutonomousRobotException(TAG, "SAX Exception " + sx.getMessage());
        } catch (IOException iex) {
            throw new AutonomousRobotException(TAG, "IOException " + iex.getMessage());
        }

        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;

       webcam1Name = this.hardwareMap.get(WebcamName.class, "Webcam 1");

        driveTrain = new DriveTrain(hardwareMap);
//        wobbleArm = new WobbleArm(hardwareMap, configXML);
//        ringShooter = new RingShooter(hardwareMap, configXML);
    }

    public void initializeIMU() {
        imu = new OptimizedIMU(hardwareMap, opMode);
    }

    public void initializeT265Camera() {

        Transform2d cameraToRobot = new Transform2d();
// Increase this value to trust encoder odometry less when fusing encoder measurements with VSLAM
        double encoderMeasurementCovariance = 0.8;
        Pose2d startingPose = new Pose2d(0, 0, new Rotation2d());
        slamra = new T265Camera(cameraToRobot, encoderMeasurementCovariance, hardwareMap.appContext);
        slamra.setPose(startingPose);
        slamra.start();
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
