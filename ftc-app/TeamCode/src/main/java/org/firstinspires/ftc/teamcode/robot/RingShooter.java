package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.ftcdevcommon.XPathAccess;
import org.firstinspires.ftc.teamcode.auto.xml.RobotConfigXML;
import org.firstinspires.ftc.teamcode.hardware.motor.LCHSMotor;
import org.firstinspires.ftc.teamcode.hardware.servo.LCHSServo;

import javax.xml.xpath.XPathExpressionException;

public class RingShooter {

    private double elevatorPower;
    private int elevatorUpPosition;
    private int elevatorDownPosition;
    private boolean elevatorIsUp = false;

    public LCHSMotor elevatorMotor;
    public LCHSMotor shootMotor;
    public LCHSMotor intakeMotor;
    public LCHSServo triggerServo;

    RingShooter(HardwareMap hardwareMap, RobotConfigXML config) {
        intakeMotor = new LCHSMotor(hardwareMap, "intake");
        shootMotor = new LCHSMotor(hardwareMap, "shoot");
        elevatorMotor = new LCHSMotor(hardwareMap, "elevator");
        triggerServo = new LCHSServo(hardwareMap, "trigger", config);

        XPathAccess elevatorConfig = config.getPath("ELEVATOR");
        try {
            elevatorPower = elevatorConfig.getDouble("power", 1);
            elevatorUpPosition = elevatorConfig.getInt("up", 0);
            elevatorDownPosition = elevatorConfig.getInt("down", 0);
        } catch (XPathExpressionException e) {
            RobotLogCommon.e("RingShooter:Elevator", "Elevator config values not found");
        }
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void toggleElevator() {
        if (elevatorIsUp)
            moveElevatorDown();
        else
            moveElevatorUp();
    }

    public void moveElevatorUp() {
        elevatorIsUp = true;
        moveElevator(elevatorUpPosition);
    }

    public void moveElevatorDown() {
        elevatorIsUp = false;
        moveElevator(elevatorDownPosition);
    }

    private void moveElevator(int position) {
        elevatorMotor.setTargetPosition(position);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorMotor.setPower(elevatorPower);
    }

}

