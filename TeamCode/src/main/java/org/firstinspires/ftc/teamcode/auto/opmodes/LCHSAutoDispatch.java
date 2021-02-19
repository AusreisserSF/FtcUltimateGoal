package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.android.WorkingDirectory;
import org.firstinspires.ftc.teamcode.robot.LCHSRobot;
import org.firstinspires.ftc.teamcode.auto.FTCAuto;
import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.RobotConstantsUltimateGoal;

// Use this dispatcher class to place the launching of LCHSAuto and all of the error
// handling in one place.
public class LCHSAutoDispatch {

    public void runOpMode(RobotConstantsUltimateGoal.OpMode pOpMode, RobotConstants.Alliance pAlliance, LinearOpMode pLinear) throws InterruptedException {

        final String TAG = pOpMode.toString();
        pLinear.telemetry.setAutoClear(false); // keep our messages on the driver station

        // LCHSAuto, the common class for all autonomous opmodes, needs
        // access to the public data fields and methods in LinearOpMode.
        try {
            RobotLogCommon.initialize(WorkingDirectory.getWorkingDirectory() + RobotConstants.logDir);
            FTCAuto runAuto = new FTCAuto(pOpMode, pAlliance, pLinear);

            // Possible fix to disconnecting phone; send telemetry (see ftc github troubleshooting)
            //waitForStart();
            while (!pLinear.opModeIsActive() && !pLinear.isStopRequested()) {
                pLinear.telemetry.addData(TAG, "Waiting for start ...");
                pLinear.telemetry.update();
                pLinear.telemetry.clearAll(); // Do this to avoid
                // E EventLoopManager: com.qualcomm.robotcore.exception.RobotCoreException: Cannot have more than 255 string data points
            }

            pLinear.telemetry.addData(TAG, "Running " + TAG + " ...");
            pLinear.telemetry.update();

            runAuto.runRobot();
        }

        // Catch clauses ---

        // Keep this comment here as a warning!
        // You can't call System.exit(1); because the robot controller then exits and
        // can't be restarted from the driver station. The result is that if you get
        // a fatal error during the autonomous run and exit you can't restart the robot
        // for the driver-controlled run.

        // NOTE: from the ftc documentation - "Please do not swallow the InterruptedException,
        // as it is used in cases where the op mode needs to be terminated early."
        catch (InterruptedException iex) {
            RobotLogCommon.d(TAG, "Caught InterruptedException; rethrowing");
            throw iex;
        }

        // For all other exceptions don't ever do this:
        // throw(arx); // propagate error
        // - rethrowing shuts down the entire application and prevents TeleOp
        // from starting.

        // The method below holds the error message on the DS screen while
        // not triggering the DS to restart the RC because of an absence of
        // communication.
        catch (AutonomousRobotException arx) {
            RobotLogCommon.d(arx.getTag(), arx.getMessage());
            while (!pLinear.isStopRequested()) {
                pLinear.telemetry.addData(TAG + " fatal error", arx.getMessage());
                pLinear.telemetry.update();
                pLinear.telemetry.clearAll(); // do not repeat the message
                android.os.SystemClock.sleep(1000);
            }
        } catch (Exception ex) {
            RobotLogCommon.d(TAG, ex.getMessage());
            while (!pLinear.isStopRequested()) {
                pLinear.telemetry.addData(TAG + " fatal Exception", ex.getMessage());
                pLinear.telemetry.update();
                pLinear.telemetry.clearAll(); // do not repeat the message
                android.os.SystemClock.sleep(1000);
            }
        } finally {
            RobotLogCommon.closeLog();
        }
    }
}
