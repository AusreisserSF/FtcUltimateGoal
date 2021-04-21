package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.xml.RobotConfigXML;
import org.firstinspires.ftc.teamcode.hardware.motor.LCHSMotor;
import org.firstinspires.ftc.teamcode.hardware.servo.LCHSServo;

import static android.os.SystemClock.sleep;


public class WobbleArm {

    public enum FlipState { //changed all the negatives to positive due to new wobble position. -Harold
        REST(0),
        FLOATING(550),
        OUT(730),
        INTAKE(770),
        IN(100),
        DROP(500);

        private final int position;
        FlipState(int position) {
            this.position = position;
        }

        public int getPosition() {
            return position;
        }
    }

    public static final double FLIP_POWER_FACTOR = 0.6;

    public LCHSMotor flipMotor;
    public LCHSServo servo;

    private FlipState flipState;

    WobbleArm(HardwareMap hardwareMap, RobotConfigXML configXML) {
        flipMotor = new LCHSMotor(hardwareMap, "wobblemotor");
        servo = new LCHSServo(hardwareMap, "wobble", configXML);
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
