package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.xml.RobotConfigXML;
import org.firstinspires.ftc.teamcode.hardware.motor.LCHSMotor;
import org.firstinspires.ftc.teamcode.hardware.servo.LCHSServo;

public class OldRingShooter {

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

    public LCHSMotor intakeMotor;
    public LCHSMotor liftMotor;
    public LCHSMotor shootMotor;
    public LCHSServo gateServo;

    private ServoState gateState;

    OldRingShooter(HardwareMap hardwareMap, RobotConfigXML config) {
        intakeMotor = new LCHSMotor(hardwareMap, "intake");
        liftMotor = new LCHSMotor(hardwareMap, "lift");
        shootMotor = new LCHSMotor(hardwareMap, "shoot");
        gateServo = new LCHSServo(hardwareMap, "gate");
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
