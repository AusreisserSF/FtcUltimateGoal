package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.motor.CachingMotorEx;
import org.firstinspires.ftc.teamcode.hardware.servo.CachingServo;

public class RingShooter {

    public enum ServoState {
        REST(0.0),
        UP(0.9),
        DOWN(0.5);

        private final double position;
        ServoState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public CachingMotorEx intakeMotor;
    public CachingMotorEx liftMotor;
    public CachingMotorEx shootMotor;
    public CachingServo gateServo;

    private ServoState gateState;

    RingShooter(HardwareMap hardwareMap) {
        intakeMotor = new CachingMotorEx(hardwareMap, "intake");
        liftMotor = new CachingMotorEx(hardwareMap, "lift");
        shootMotor = new CachingMotorEx(hardwareMap, "shoot");
        gateServo = new CachingServo(hardwareMap, "gate");
    }

    public void shootPowerShot() {
        // TODO: implement shoot target
    }

    public ServoState getServoState() {
        return gateState;
    }

    public void setServoState(ServoState state) {
        gateState = state;
        gateServo.setPosition(state.getPosition());
    }

}
