package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

//** TEMP import org.firstinspires.ftc.ftcappcommon.AutonomousRobotException;

// Common hardware definitions for LCHSAuto and LCHSTeleOp.
public class LCHSHardwareMap {

    private static final String TAG = "LCHSHardwareMap";

    /* Public OpMode members. */
    public final DcMotor leftFrontDrive;
    public final DcMotor rightFrontDrive;
    public final DcMotor leftBackDrive;
    public final DcMotor rightBackDrive;

    public static final double NORMAL_POWER = 1.0;
    public static final double REDUCED_POWER_50 = 0.5;
    public final static double REDUCED_POWER_20 = 0.2;
    public final static double REDUCED_POWER_9 = 0.09;

    public static BNO055IMU gyro;
    public static boolean imuInitialized = false; // global IMU status (see FTCAutoDispatch)

    public final WebcamName webcam1Name;

    public enum MOTORS {leftFrontMotor, rightFrontMotor, rightBackMotor, leftBackMotor}
    public DcMotor[] motorArray = new DcMotor[MOTORS.values().length];

    public final HardwareMap hwMap;

    public LCHSHardwareMap(HardwareMap hwm, boolean pInitializeIMU) {
        hwMap = hwm;
        imuInitialized = false;
        gyro = null;

        webcam1Name = hwMap.get(WebcamName.class, "Webcam 1");

        // Define and Initialize Motors
        leftFrontDrive = hwMap.get(DcMotor.class, "left_front_drive");
        rightFrontDrive = hwMap.get(DcMotor.class, "right_front_drive");
        leftBackDrive = hwMap.get(DcMotor.class, "left_back_drive");
        rightBackDrive = hwMap.get(DcMotor.class, "right_back_drive");

        // ---------- Set Zero Power Behavior to BRAKE ----------
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorArray[MOTORS.leftFrontMotor.ordinal()] = leftFrontDrive;
        motorArray[MOTORS.rightFrontMotor.ordinal()] = rightFrontDrive;
        motorArray[MOTORS.rightBackMotor.ordinal()] = rightBackDrive;
        motorArray[MOTORS.leftBackMotor.ordinal()] = leftBackDrive;

        // Set up the parameters with which we will use our IMU. Note that integration
        // algorithm here just reports accelerations to the logcat log; it doesn't actually
        // provide positional information.
        if (pInitializeIMU) {
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
            //parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
            parameters.loggingEnabled = true;
            parameters.loggingTag = "imu";
            parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
            parameters.mode = BNO055IMU.SensorMode.IMU;

            // Retrieve and initialize the IMU.
            RobotLog.dd(TAG, "Mapping imu");
            gyro = hwMap.get(BNO055IMU.class, "imu");
            RobotLog.dd(TAG, "initing the imu parameters");
            gyro.initialize(parameters);
            RobotLog.dd(TAG, "Finished initing the imu parameters");

            // The ftc library internally retries the IMU if the status is not RUNNING_FUSION
            // so do not retry initialization. But we will retry getting the status.
            for (int i = 0; i < 3; i++) {
                BNO055IMU.SystemStatus localIMUStatus = gyro.getSystemStatus();
                RobotLog.dd(TAG, "imu status after init, read count " + i +", status " + localIMUStatus);
                if (localIMUStatus == BNO055IMU.SystemStatus.RUNNING_FUSION) {
                    imuInitialized = true;
                    break;
                }
                android.os.SystemClock.sleep(500);
            }

            if (!imuInitialized)
                //** TEMP throw new AutonomousRobotException(TAG, "Failed to initialize IMU");
                RobotLog.d("Failed to initialize IMU");
        }

        // NeverRest20
        /*
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        */

        // NeverRest 40
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);

        // RUN_USING_ENCODERS means that
        // "The motor is to do its best to run at targeted velocity."
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }
}
