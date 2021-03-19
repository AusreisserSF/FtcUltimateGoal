package org.firstinspires.ftc.ftcdevcommon.android.uvc;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathFactory;

// XML configuration file for UVC webcam devices.
public class UVCCameraConfigurationXML {

    public static final String TAG = "UVCCameraConfigurationXML";

    private final XPath xpath;
    private final NodeList cameraNodes;
    private final int cameraNodeCount;

    public UVCCameraConfigurationXML(String pXMLFilePath) {

        try {
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setIgnoringComments(true);

            // ONLY works with a validating parser (DTD or schema) dbFactory.setIgnoringElementContentWhitespace(true);
            // Crashes on Android dbFactory.setXIncludeAware(true);

            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            Document document = dBuilder.parse(new File(pXMLFilePath));

            cameraNodes = document.getElementsByTagName("uvc_camera");
            cameraNodeCount = cameraNodes.getLength();
            RobotLogCommon.d(TAG, "In UVCCameraConfigurationXML; opened and parsed the XML configuration file");
            RobotLogCommon.d(TAG, "Found " + cameraNodeCount + " cameras");

            XPathFactory xpathFactory = XPathFactory.newInstance();
            xpath = xpathFactory.newXPath();

        } catch (ParserConfigurationException | SAXException | IOException ex) {
            throw new AutonomousRobotException(TAG, "Exception in XML processing " + ex.getMessage());
        }
    }

    // Iterate through the top-level elements of the
    // UVCCameraConfig.xml file, find the requested camera,
    // create a UVCCamera class for each camera, and return them.
    public List<RobotCameraUVC> getUVCCameras() {

        List<RobotCameraUVC> cameras = new ArrayList<>();

        // Find the XML element that matches the requested camera.
        Node oneCameraNode;
        UVCCameraXML uvcCameraXML;
        for (int i = 0; i < cameraNodeCount; i++) {
            oneCameraNode = cameraNodes.item(i);

            if (oneCameraNode.getNodeType() != Node.ELEMENT_NODE)
                continue;

            // Collect all the information about a single camera.
            uvcCameraXML = new UVCCameraXML(xpath, (Element) oneCameraNode, ((Element) oneCameraNode).getTagName());
            cameras.add(uvcCameraXML.getOneCameraElement());
        }

        return cameras;
    }


}
