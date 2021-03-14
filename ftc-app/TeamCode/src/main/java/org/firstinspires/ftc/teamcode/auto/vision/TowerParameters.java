package org.firstinspires.ftc.teamcode.auto.vision;

// Parameters extracted from RingParameters.xml.
public class TowerParameters {

    public final CommonParameters.ImageParameters imageParameters;
    public final CommonParameters.GrayParameters grayParameters;
  
    public TowerParameters(CommonParameters.ImageParameters pImageParameters, CommonParameters.GrayParameters pGrayParameters) {
        imageParameters = pImageParameters;
        grayParameters = pGrayParameters;
    }
}