package org.firstinspires.ftc.teamcode.hardware.imu;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ReadWriteFile;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.robot.utils.RobotRunnable;
import org.firstinspires.ftc.teamcode.math.Angle;
import org.firstinspires.ftc.teamcode.math.LCHSMath;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicReference;

public class OptimizedIMU {

    private final List<BNO055IMU> delegates;
    private final HeadingIntegrator headingIntegrator;

    public OptimizedIMU(HardwareMap hardwareMap, LinearOpMode opMode) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";

        delegates = new ArrayList<>();
        List<LynxModule> modules = hardwareMap.getAll(LynxModule.class);
        for (int i = 0; i < modules.size(); i++) {
            LynxModule module = modules.get(i);
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
            BNO055IMU imu = new LynxEmbeddedIMU(OptimizedI2cDevice.createLynxI2cDeviceSynch(module, 0));
//            parameters.calibrationDataFile = "BNO055IMUCalibration" + i + ".json";
            imu.initialize(parameters);
            delegates.add(imu);
        }

        headingIntegrator = new HeadingIntegrator(opMode);
        headingIntegrator.start();
    }

    public Angle getHeading() {
        return new Angle(AngleUnit.normalizeDegrees(getIntegratedHeading().getDegrees()), AngleUnit.DEGREES);
    }

    public Angle getIntegratedHeading() {
        return new Angle(headingIntegrator.getDegrees(), AngleUnit.DEGREES);
    }

    public void calibrate() {
        for (int i = 0; i < delegates.size(); i++) {
            BNO055IMU imu = delegates.get(i);
            if (imu.isGyroCalibrated()) {
                BNO055IMU.CalibrationData calibrationData = imu.readCalibrationData();
                String filename = "BNO055IMUCalibration" + i + ".json";
                File file = AppUtil.getInstance().getSettingsFile(filename);
                ReadWriteFile.writeFile(file, calibrationData.serialize());
                RobotLog.dd("OptimizedIMU:CalibrationFile" + i, filename);
            }
        }
    }

    class HeadingIntegrator extends RobotRunnable {
        private AtomicReference<Double> cumulativeDegrees = new AtomicReference<>();
        private int numberOfImus;
        private List<Number> currentAngles;
        private List<Number> integratedAngles;

        HeadingIntegrator(LinearOpMode opMode) {
            this.opMode = opMode;
        }

        double getDegrees() {
            return cumulativeDegrees.get();
        }

        @Override
        protected void onRun() {
            RobotLog.dd("Optimized IMU", "Starting integrator");
            cumulativeDegrees.set(0.0);
            numberOfImus = delegates.size();
            currentAngles = new ArrayList<>();
            integratedAngles = new ArrayList<>();
            for (BNO055IMU delegate : delegates) {
                currentAngles.add(delegate.getAngularOrientation().firstAngle);
                integratedAngles.add(0);

            }
        }

        @Override
        protected void insideRun() {
            List<Number> previousAngles = new ArrayList<>(currentAngles);
            for (int i = 0; i < numberOfImus; i++) {
                currentAngles.set(i, delegates.get(i).getAngularOrientation().firstAngle);
                double deltaDegrees = currentAngles.get(i).doubleValue() - previousAngles.get(i).doubleValue();
                if (deltaDegrees > 180) {
                    deltaDegrees -= 360;
                } else if (deltaDegrees < -180) {
                    deltaDegrees += 360;
                }
                integratedAngles.set(i, integratedAngles.get(i).doubleValue() + deltaDegrees);
            }
            cumulativeDegrees.set( LCHSMath.mean(integratedAngles) );
        }

        @Override
        protected boolean runIsComplete() {
            return false;
        }

        @Override
        protected void onEndRun() {
            RobotLog.dd("OptimizedIMU", "Stopped thread");
        }
    }

}
