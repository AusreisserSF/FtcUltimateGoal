package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.xml.RobotConfigXML;
import org.firstinspires.ftc.teamcode.hardware.motor.CachingMotorEx;
import org.firstinspires.ftc.teamcode.hardware.servo.LCHSServo;

public class RingShooter2 {

    //**TODO need actual positions
    public enum ServoState {
        REST(0.0),
        UP(0.9),
        DOWN(0.6);

        private final double position;
        ServoState(double position) {
            this.position = position;
        }

        public double getPosition() {
            return position;
        }
    }

    public static final double ELEVATOR_POWER_FACTOR = 0.6;

    public CachingMotorEx elevatorMotor;
    public CachingMotorEx shootMotor;
    public LCHSServo shooterTriggerServo;

    private ServoState triggerState;

    RingShooter2(HardwareMap hardwareMap, RobotConfigXML config) {
        elevatorMotor = new CachingMotorEx(hardwareMap, "elevator");
        shootMotor = new CachingMotorEx(hardwareMap, "shoot");
        shooterTriggerServo = new LCHSServo(hardwareMap, "trigger");
    }

    public void shootPowerShot() {
        // TODO: implement shoot target
    }

    public ServoState getServoState() {
        return triggerState;
    }

    public void setServoState(ServoState state) {
        triggerState = state;
        shooterTriggerServo.setPosition(state.getPosition());
    }

}

