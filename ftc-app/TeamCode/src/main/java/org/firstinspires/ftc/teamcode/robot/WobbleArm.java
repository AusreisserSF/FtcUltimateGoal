package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.motor.CachingMotorEx;
import org.firstinspires.ftc.teamcode.hardware.servo.CachingServo;

public class WobbleArm {

    public enum ServoState {
        REST(0.0),
        HOLD(0.6),
        RELEASE(0.1);

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
        HOLD(4000),
        RELEASE(1000);

        private final int position;
        FlipState(int position) {
            this.position = position;
        }

        public int getPosition() {
            return position;
        }
    }

    public static final double FLIP_POWER_FACTOR = 1.0;

    public CachingMotorEx flipMotor;
    public CachingServo servo;

    private ServoState servoState;
    private FlipState flipState;

    WobbleArm(HardwareMap hardwareMap) {
        flipMotor = new CachingMotorEx(hardwareMap, "wobble motor");
        servo = new CachingServo(hardwareMap, "wobble servo");
        flipMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
    }
}
