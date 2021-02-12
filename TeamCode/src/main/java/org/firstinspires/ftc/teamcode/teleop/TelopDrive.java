/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teleop;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.LCHSHardwareMap;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.List;

/**
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the LCHSHardwareMap class.
 * The code is structured as a LinearOpMode
 * <p>
 * This particular OpMode executes a POV Game style Teleop for a PushBot
 * In this mode the left stick moves the robot FWD and back, the Right stick turns left and right.
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "Telop Drive", group = "Pushbot")
//@Disabled
public class TelopDrive extends LinearOpMode {

    //Logging tag identifier
    private static final String TAG = "TelopDrive ";

    /* Declare OpMode members. */
    double[] motorValues = new double[LCHSHardwareMap.MOTORS.values().length];
    LCHSHardwareMap robot;

    @Override
    public void runOpMode() {

        robot = new LCHSHardwareMap(hardwareMap, false);
 
        //Set reference time
        long startTime = System.currentTimeMillis();
        boolean firstIteration = true;

        //Fields for driver control mode
        double forward = 0;
        double strafe = 0;
        double rotate = 0;

        //powerModifier changes the gamepad input from 100% to something less.  This is done to
        //help the drivers by reducing the sensitivity of the imput.  Using this technique
        //provides for more control
        double powerModifier = LCHSHardwareMap.NORMAL_POWER;

        // Send telemetry message to signify robot waiting;
        telemetry.addData(" ", "Hello Driver");    //
        telemetry.addData(">", "Robot Ready.");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        Log.d(TAG, "Waiting to start");
        waitForStart();

        DecimalFormat fmt = new DecimalFormat("#########0.000");
        double desiredAngle = 0.0;
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            forward = -gamepad1.right_stick_y * powerModifier;
            strafe = gamepad1.left_stick_x * powerModifier;
            rotate = gamepad1.right_stick_x * powerModifier;
            //Log.d(TAG, "Stick Inputs: forward: " + fmt.format(forward) + ", strafe: " + fmt.format(strafe) + ", rotate: " + fmt.format(rotate));
            // 19-Nov-2019 Do not use gyro in TeleOp (same as 2018-19): we get IMU
            // errors when we attempt to initialize the IMU in Autonomous and then
            // again in TeleOp.
            desiredAngle = guideRobot(false, 0.005, desiredAngle, forward, strafe, rotate);

            //Set the power modes
            if (gamepad1.y) powerModifier = LCHSHardwareMap.NORMAL_POWER;
            if (gamepad1.b) powerModifier = LCHSHardwareMap.REDUCED_POWER_50;
            //else if (gamepad1.right_trigger != 0.0)
            //    powerModifier = LCHSHardwareMap.REDUCED_POWER_20;
            //else if (gamepad1.left_trigger != 0.0) powerModifier = LCHSHardwareMap.REDUCED_POWER_9;

            telemetry.update();
            sleep(20);
        }
    }

    /*
    This routine has been reduced to basically after the power correction has proven to be of no help
     */
    public void setPowerLevels(double p[], DcMotor[] ma) {
        for (int i = 0; i < p.length; i++) {
            ma[i].setPower(p[i]);
            //Log.d(TAG, "Motor " + i + " power level " + p[i]);
        }
    }

    /* normalizePowerLevels -- normalizes the power levels so that the never exceed +- 1
     */
    public double[] normalizePowerLevels(double p[]) {
        double max = 0;
        for (int i = 0; i < p.length; i++) {
            if (max < Math.abs(p[i]))
                max = Math.abs(p[i]);
        }
        if (max > 1.0) {
            for (int i = 0; i < p.length; i++) {
                p[i] /= max;
            }
        }
        return p;
    }

    /*
    guideRobot  --  Inplements the gyro assisted steering for macanum drive
     */
    public double guideRobot(boolean useGyro, double gyroGain, double desiredAngle,
                             double forward, double strafe, double rotate) {
        if (useGyro && (rotate == 0.0)) {
            rotate = -getError(desiredAngle) * gyroGain;
        } else {
            if (useGyro && (rotate != 0.0)) {
                desiredAngle = getAngle();
            }
        }
        motorValues[LCHSHardwareMap.MOTORS.leftFrontMotor.ordinal()] = strafe + forward + rotate;
        motorValues[LCHSHardwareMap.MOTORS.rightFrontMotor.ordinal()] = -strafe + forward - rotate;
        motorValues[LCHSHardwareMap.MOTORS.rightBackMotor.ordinal()] = strafe + forward - rotate;
        motorValues[LCHSHardwareMap.MOTORS.leftBackMotor.ordinal()] = -strafe + forward + rotate;
        setPowerLevels(normalizePowerLevels(motorValues), robot.motorArray);
        return desiredAngle;
    }

    /**
     * headingDelta determines the error between the target angle and the robot's current heading
     *
     * @param targetAngle Desired angle (relative to global reference established at last Gyro Reset).
     * @return error angle: Degrees in the range +/- 180. Centered on the robot's frame of reference
     * +ve error means the robot should turn LEFT (CCW) to reduce error.
     */
    public double getError(double targetAngle) {

        double robotError;

        // calculate error in -179 to +180 range  (
        robotError = targetAngle - getAngle();
        while (robotError > 180) robotError -= 360;
        while (robotError <= -180) robotError += 360;
        return robotError;
    }


    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = robot.gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }
}

