package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.teamcode.auto.RobotConstants;
import org.firstinspires.ftc.teamcode.auto.RobotConstantsUltimateGoal;

// Holds the results of image recognition.
public class RingReturn {

    public final boolean fatalComputerVisionError;
    public final RobotConstantsUltimateGoal.TargetZone targetZone;

    public RingReturn(boolean pFatalComputerVisionError, RobotConstantsUltimateGoal.TargetZone pTargetZone) {
        fatalComputerVisionError = pFatalComputerVisionError;
        targetZone = pTargetZone;
    }

}

