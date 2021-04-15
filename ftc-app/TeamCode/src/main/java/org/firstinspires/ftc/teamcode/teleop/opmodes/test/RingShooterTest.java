package org.firstinspires.ftc.teamcode.teleop.opmodes.test;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.teleop.utility.Button;
import org.firstinspires.ftc.teamcode.teleop.utility.TeleOpBase;
import org.firstinspires.ftc.teamcode.robot.RingShooter;


@TeleOp(group="Test")
//@Disabled
public class RingShooterTest extends TeleOpBase {

    private final Button shootButton = new Button();
    private final Button goShoot = new Button();
    private final Button elevatorButton = new Button();

    @Override
    protected void initialize() {
        telemetry.addData("elevator", "right and left triggers");
        telemetry.addData("shoot", "right stick y");
        telemetry.addData("shoot velocity", robot.shooter.shootMotor.getVelocity());
        telemetry.update();
    }

    @Override
    protected void update() {

        shootButton.update(gamepad1.x);
        goShoot.update(gamepad1.a);
        elevatorButton.update(gamepad1.b);

        if (goShoot.is(Button.State.TAP)){
            if (gamepad1.right_bumper) {
                robot.shooter.shootMotor.setVelocity(0);
            } else if (gamepad1.left_bumper){
                robot.shooter.shootMotor.setVelocity(2000);
            } else {
                robot.shooter.shootMotor.setVelocity(2500);
            }
        }

        if (shootButton.is(Button.State.TAP)) {
            robot.shooter.triggerServo.setState("out");
            sleep(50);
            robot.shooter.triggerServo.setState("rest");
        }

        if (elevatorButton.is(Button.State.TAP)){
            robot.shooter.toggleElevator();
        }

        telemetry.addData("elevator target pos", robot.shooter.elevatorMotor.getTargetPosition());
        telemetry.addData("elevator actual pos", robot.shooter.elevatorMotor.getCurrentPosition());
        telemetry.addData("elevator power", robot.shooter.elevatorMotor.getPower());
        telemetry.addData("shoot velocity", robot.shooter.shootMotor.getVelocity());
        telemetry.update();

    }

}

