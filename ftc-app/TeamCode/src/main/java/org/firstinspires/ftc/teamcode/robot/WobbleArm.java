package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.xml.RobotConfigXML;
import org.firstinspires.ftc.teamcode.hardware.motor.CachingMotorEx;
import org.firstinspires.ftc.teamcode.hardware.servo.CachingServo;

import static android.os.SystemClock.sleep;


public class WobbleArm {

    public enum ServoState {
        REST(0.4),
        HOLD(0.9),
        RELEASE(0.2);

        private final double position;
        ServoState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public enum FlipState {
        REST(0),
        OUT(-730),
        INTAKE(-770),
        IN(0);

        private final int position;
        FlipState(int position) {
            this.position = position;
        }

        public int getPosition() {
            return position;
        }
    }

    public static final double FLIP_POWER_FACTOR = 0.6;

    public CachingMotorEx flipMotor;
    public CachingServo servo;

    private ServoState servoState;
    private FlipState flipState;

    WobbleArm(HardwareMap hardwareMap, RobotConfigXML configXML) {
        flipMotor = new CachingMotorEx(hardwareMap, "wobble motor");
        servo = new CachingServo(hardwareMap, "wobble servo");

       //**TODO Not in the xml file!! configXML.getPath("servo");
    }

    public ServoState getServoState() {
        return servoState;
    }

    public void setServoState(ServoState state) {
        servoState = state;
        servo.setPosition(state.getPosition());
    }

    public FlipState getFlipState() {
        return flipState;
    }

    public void setFlipState(FlipState state) {
        flipState = state;
        flipMotor.setTargetPosition(state.getPosition());
        flipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        flipMotor.setPower(FLIP_POWER_FACTOR);
    }

    public void waitForFlip() {
        while (flipMotor.isBusy()) {
            sleep(20);
        }
    }
}
