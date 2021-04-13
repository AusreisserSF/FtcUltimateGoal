// Port of XMLCommon.cpp
package org.firstinspires.ftc.teamcode.auto.xml;

import org.w3c.dom.Node;
import org.opencv.core.Rect;

import org.firstinspires.ftc.teamcode.auto.vision.CommonParameters;
import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;

public class ImageXMLCommon {

    public static final String TAG = "CommonParametersXML";

    // Parse the XML elements that describe the image to be analyzed.
/*
<image_parameters>
<file></file>
<resolution>
	<width></width>
	<height></height>
</resolution>
<image_roi>
	<x></x>
	<y></y>
	<width></width>
	<height></height>
</image_roi>
</image_parameters>
*/
// Parse the children of the <image_parameters> element in the XML file.
    public CommonParameters.ImageParameters parseImageParameters(Node pImageParametersNode) {
        String file_name;
        int resolution_width;
        int resolution_height;
        Rect image_roi;

        RobotLogCommon.d(TAG, "Parsing XML image_parameters");

        Node file_node = pImageParametersNode.getFirstChild();
        file_node = getNextElement(file_node);
        if ((file_node == null) || !file_node.getNodeName().equals("file"))
            throw new AutonomousRobotException(TAG, "Element 'file' not found");

        // The file_name element may be empty, and certainly won't be used, when we're taking images from a camera.
        file_name = file_node.getTextContent();

	/*
	<resolution>
		<width></width>
		<height></height>
	</resolution>
	*/
        Node resolution_node = file_node.getNextSibling();
        resolution_node = getNextElement(resolution_node);
        if ((resolution_node == null) || !resolution_node.getNodeName().equals("resolution"))
            throw new AutonomousRobotException(TAG, "Element 'resolution' not found");

        // Get the two children of the resolution node: width and height
        Node resolution_width_node = resolution_node.getFirstChild();
        resolution_width_node = getNextElement(resolution_width_node);
        if ((resolution_width_node == null) || !resolution_width_node.getNodeName().equals("width") || resolution_width_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'resolution/width' missing or empty");

        try {
            resolution_width = Integer.parseInt(resolution_width_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'resolution/width'");
        }

        Node resolution_height_node = resolution_width_node.getNextSibling();
        resolution_height_node = getNextElement(resolution_height_node);
        if ((resolution_height_node == null) || !resolution_height_node.getNodeName().equals("height") || resolution_height_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'resolution/height' missing or empty");

        try {
            resolution_height = Integer.parseInt(resolution_height_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'resolution/height'");
        }

        // Parse the region of interest parameters.
        Node image_roi_node = resolution_node.getNextSibling();
        image_roi_node = getNextElement(image_roi_node);
        if ((image_roi_node == null) || !image_roi_node.getNodeName().equals("image_roi"))
            throw new AutonomousRobotException(TAG, "Element 'image_roi' not found");

        image_roi = parseROI(image_roi_node);

        return new CommonParameters.ImageParameters(file_name, resolution_width, resolution_height, image_roi);
    }

    // Parse any element that contains the 4 ROI children.
/*
<!-- any node -->
	<x></x>
	<y></y>
	<width></width>
	<height></height>
*/
    public static Rect parseROI(Node pROINode) {
        Rect roiElement = new Rect();

        Node roi_x_node = pROINode.getFirstChild();
        roi_x_node = getNextElement(roi_x_node);
        if ((roi_x_node == null) || !roi_x_node.getNodeName().equals("x") || roi_x_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'x' missing or empty");

        try {
            roiElement.x = Integer.parseInt(roi_x_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'x'");
        }

        Node roi_y_node = roi_x_node.getNextSibling();
        roi_y_node = getNextElement(roi_y_node);
        if ((roi_y_node == null) || !roi_y_node.getNodeName().equals("y") || roi_y_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'y' missing or empty");

        try {
            roiElement.y = Integer.parseInt(roi_y_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'y'");
        }

        // Get the width and height elements
        Node roi_width_node = roi_y_node.getNextSibling();
        roi_width_node = getNextElement(roi_width_node);
        if ((roi_width_node == null) || !roi_width_node.getNodeName().equals("width") || roi_width_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'width' missing or empty");

        try {
            roiElement.width = Integer.parseInt(roi_width_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'width'");
        }

        Node roi_height_node = roi_width_node.getNextSibling();
        roi_height_node = getNextElement(roi_height_node);
        if ((roi_height_node == null) || !roi_height_node.getNodeName().equals("height") || roi_height_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'height' missing or empty");

        try {
            roiElement.height = Integer.parseInt(roi_height_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'height'");
        }

        return roiElement;
    }

    // Parse the children of the <gray_parameters> element in the XML file.
    public CommonParameters.GrayParameters parseGrayParameters(Node pGrayNode) {
        int target;
        int low_threshold;

        RobotLogCommon.d(TAG, "Parsing XML gray_parameters");
        Node gray_target_node = pGrayNode.getFirstChild();
        gray_target_node = getNextElement(gray_target_node);
        if ((gray_target_node == null) || !gray_target_node.getNodeName().equals("target") || gray_target_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'target' missing or empty");

        try {
            target = Integer.parseInt(gray_target_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'target'");
        }

        Node gray_threshold_node = gray_target_node.getNextSibling();
        gray_threshold_node = getNextElement(gray_threshold_node);
        if ((gray_threshold_node == null) || !gray_threshold_node.getNodeName().equals("low_threshold") || gray_threshold_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'low_threshold' missing or empty");

        try {
            low_threshold = Integer.parseInt(gray_threshold_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'low_threshold'");
        }

        return new CommonParameters.GrayParameters(target, low_threshold);
    }


// Parse the children of the <hsv_parameters> element in the XML file.
/*
<hsv_parameters>
    <hue_name>gold</hue_name>
    <low_hue>10</low_hue>
    <high_hue>30</high_hue>
	<saturation_target>200</saturation_target>
	<saturation_low_threshold>165</saturation_low_threshold>
	<value_target>200</value_target>
    <value_low_threshold>180</value_low_threshold>
</hsv_parameters>
*/
    // At this point pHSVNode points to the <hsv_parameters> element.
    public CommonParameters.HSVParameters parseHSVParameters(Node pHSVNode) {
        String hue_name;
        int hue_low;
        int hue_high;
        int saturation_target;
        int saturation_low_threshold;
        int value_target;
        int value_low_threshold;

        RobotLogCommon.d(TAG, "Parsing XML hsv_parameters");
        Node hue_name_node = pHSVNode.getFirstChild();
        hue_name_node = getNextElement(hue_name_node);
        if ((hue_name_node == null) || !hue_name_node.getNodeName().equals("hue_name") || hue_name_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'hue_name' missing or empty");

        hue_name = hue_name_node.getTextContent();

        Node hue_low_node = hue_name_node.getNextSibling();
        hue_low_node = getNextElement(hue_low_node);
        if ((hue_low_node == null) || !hue_low_node.getNodeName().equals("hue_low") || hue_low_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'hue_low' missing or empty");

        try {
            hue_low = Integer.parseInt(hue_low_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'hue_low'");
        }

        Node hue_high_node = hue_low_node.getNextSibling();
        hue_high_node = getNextElement(hue_high_node);
        if ((hue_high_node == null) || !hue_high_node.getNodeName().equals("hue_high") || hue_high_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'hue_high' missing or empty");
        try {
            hue_high = Integer.parseInt(hue_high_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'hue_high'");
        }

        // <saturation_target>
        Node saturation_target_node = hue_high_node.getNextSibling();
        saturation_target_node = getNextElement(saturation_target_node);
        if ((saturation_target_node == null) || !saturation_target_node.getNodeName().equals("saturation_target") || saturation_target_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'saturation_target' missing or empty");
        try {
            saturation_target = Integer.parseInt(saturation_target_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'saturation_target'");
        }

        // <saturation_low_threshold>
        Node saturation_low_threshold_node = saturation_target_node.getNextSibling();
        saturation_low_threshold_node = getNextElement(saturation_low_threshold_node);
        if ((saturation_low_threshold_node == null) || !saturation_low_threshold_node.getNodeName().equals("saturation_low_threshold") || saturation_low_threshold_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'saturation_low_threshold' missing or empty");
        try {
            saturation_low_threshold = Integer.parseInt(saturation_low_threshold_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'saturation_low_threshold'");
        }

        // <value_target>
        Node value_target_node = saturation_low_threshold_node.getNextSibling();
        value_target_node = getNextElement(value_target_node);
        if ((value_target_node == null) || !value_target_node.getNodeName().equals("value_target") || value_target_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'value' missing or empty");

        try {
            value_target = Integer.parseInt(value_target_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'value_target'");
        }

        // <value_low_threshold>
        Node value_low_threshold_node = value_target_node.getNextSibling();
        value_low_threshold_node = getNextElement(value_low_threshold_node);
        if ((value_low_threshold_node == null) || !value_low_threshold_node.getNodeName().equals("value_low_threshold") || value_low_threshold_node.getTextContent().isEmpty())
            throw new AutonomousRobotException(TAG, "Element 'value_low_threshold' missing or empty");

        try {
            value_low_threshold = Integer.parseInt(value_low_threshold_node.getTextContent());
        } catch (NumberFormatException nex) {
            throw new AutonomousRobotException(TAG, "Invalid number format in element 'value_low_threshold'");
        }

        return new CommonParameters.HSVParameters(hue_name, hue_low, hue_high,
                saturation_target, saturation_low_threshold,
                value_target, value_low_threshold);
    }

    public static Node getNextElement(Node pNode) {
        Node nd = pNode;
        while (nd != null) {
            if (nd.getNodeType() == Node.ELEMENT_NODE) {
                return nd;
            }
            nd = nd.getNextSibling();
        }
        return null;
    }

}
