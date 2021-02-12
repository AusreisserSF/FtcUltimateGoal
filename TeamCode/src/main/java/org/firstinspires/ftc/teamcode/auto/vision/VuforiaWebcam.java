package org.firstinspires.ftc.teamcode.auto.vision;

import org.firstinspires.ftc.ftcdevcommon.CommonUtils;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import java.io.IOException;
import java.util.concurrent.Callable;
import java.util.concurrent.CompletableFuture;

public class VuforiaWebcam {

    private final CompletableFuture<VuforiaLocalizer> futureVuforiaLocalizer;

    public VuforiaWebcam(WebcamName pWebcamName) {
        // Start up Vuforia initialization as a CompletableFuture.
        futureVuforiaLocalizer = CommonUtils.launchAsync(new VuforiaInitCallable(pWebcamName));
     }

    // Because Vuforia initialization is time-consuming (about 2 sec. from the logs)
    // run it in a separate thread.
    private static class VuforiaInitCallable implements Callable<VuforiaLocalizer> {

        private final WebcamName webcamName;

        VuforiaInitCallable(WebcamName pWebcamName) {
            webcamName = pWebcamName;
        }

        public VuforiaLocalizer call() {
            /*
             * To start up Vuforia, tell it the view that we wish to use for camera monitor (on the RC phone);
             * If no camera monitor is desired, use the parameterless constructor instead (commented out below).
             */
            //int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            //VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

            // OR...  Do Not Activate the Camera Monitor View, to save power
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
            parameters.vuforiaLicenseKey = "AfPN50f/////AAABmQbVFYReEUYPvJ20HhITf/lqyU7jjqG5dXYslcBiHlMEkDndbYl9BbFUAJw2eVVuY2Mw4/9Rqxhg042agEBjifHpgNxEJhdjGPGdfbj8R8JeCzEVkMiH5vV/FNZjzKO35sKnGGwpI3xVy2d5ts5TbQjbcV0PccdHUim4NHVQcFCmsBhVuUJJ4LSk6X5SluIETpDdMHpGX4VCPCMGpBdB9e/OBeXMoI65fJBbe4xpQv8WNNH61ULZl3mAks3O9ttVWrkRjBDhnvEL0ii0msL/tGkcXlDkzw0Lj3IlzYW2QUWKJsIpS6PnG2byW5WtjLwXHKkNUbcWijJ592yiRhnJ8W8nV6ThHT8NGwlpMzDpfJIf";
            parameters.cameraName = webcamName;

            /*
              Instantiate the Vuforia engine
             */
            VuforiaLocalizer vuforia1 = ClassFactory.getInstance().createVuforia(parameters);

            // Comment modified from ConceptVuforiaNavigationWebcam.
            /*
             * Because this opmode processes frames in order to pass them to OpenCV, we tell Vuforia
             * that we want to ensure that certain frame formats are available in the {@link Frame}s we
             * see.
             */
            vuforia1.enableConvertFrameToBitmap();
            return vuforia1;
        }
    }

    // Wait for Vuforia initialization to complete.
    public VuforiaLocalizer waitForVuforiaInitialization() throws IOException, InterruptedException {
        return CommonUtils.getFutureCompletion(futureVuforiaLocalizer);
    }

}
