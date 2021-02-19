package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.imu.OptimizedIMU;

import java.util.List;

//!! LCHSHardwareMap -> LCHSRobot
// Common hardware definitions for LCHSAuto and LCHSTeleOp.
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
    public DriveTrain driveTrain;
    public OptimizedIMU imu;
    public final WebcamName webcam1Name;

    private static final String TAG = "LCHSRobot";


    public LCHSRobot(LinearOpMode opMode) {
        this.opMode = opMode;
        this.hardwareMap = opMode.hardwareMap;

        webcam1Name = this.hardwareMap.get(WebcamName.class, "Webcam 1");
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
