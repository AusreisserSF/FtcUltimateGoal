package org.firstinspires.ftc.teamcode.auto.xml;

import org.firstinspires.ftc.ftcdevcommon.AutonomousRobotException;
import org.firstinspires.ftc.ftcdevcommon.RobotLogCommon;
import org.firstinspires.ftc.teamcode.auto.RobotConstantsUltimateGoal;
import org.w3c.dom.Document;
import org.w3c.dom.Element;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;

import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import javax.xml.xpath.XPath;
import javax.xml.xpath.XPathConstants;
import javax.xml.xpath.XPathExpressionException;
import javax.xml.xpath.XPathFactory;
import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

// Class whose job it is to read an XML file that contains all of the simulator
// commands needed to move the robot from each of the four possible starting
// positions to each of the possible target zones.
public class TargetZoneXML {
    public static final String TAG = "TargetZoneXML";
    private static final String FILE_NAME = "TargetZones.xml";

    /*
    // IntelliJ only
    private static final String JAXP_SCHEMA_LANGUAGE =
            "http://java.sun.com/xml/jaxp/properties/schemaLanguage";
    private static final String W3C_XML_SCHEMA =
            "http://www.w3.org/2001/XMLSchema";
     */

    private final Document document;
    private final XPath xpath;

    public TargetZoneXML(String pWorkingDirectory) {

        try {

/*
// IntelliJ only
        DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
        dbFactory.setIgnoringComments(true);
        dbFactory.setNamespaceAware(true);
        dbFactory.setValidating(true);
        dbFactory.setAttribute(JAXP_SCHEMA_LANGUAGE, W3C_XML_SCHEMA);

        //## ONLY works with a validating parser.
        dbFactory.setIgnoringElementContentWhitespace(true);
 */
            // Android only
            DocumentBuilderFactory dbFactory = DocumentBuilderFactory.newInstance();
            dbFactory.setIgnoringComments(true);
            // ONLY works with a validating parser (DTD or schema) dbFactory.setIgnoringElementContentWhitespace(true);
            //PY 8/17/2019 Android throws UnsupportedOperationException dbFactory.setXIncludeAware(true);
            // End Android only

            DocumentBuilder dBuilder = dbFactory.newDocumentBuilder();
            String tzFilename = pWorkingDirectory + FILE_NAME;
            document = dBuilder.parse(new File(tzFilename));

            XPathFactory xpathFactory = XPathFactory.newInstance();
            xpath = xpathFactory.newXPath();

            // Disallow nesting of "EXECUTE_TARGET_ZONE_STEPS"
            //String executePath = "//EXECUTE_TARGET_ZONE_STEPS"; // anywhere in the document
            //Node targetZonesNode = (Node) xpath.evaluate(executePath, document, XPathConstants.NODE);
           // if (targetZonesNode != null)
           //     throw new AutonomousRobotException(TAG, "Nested EXECUTE_TARGET_ZONE_STEPS not allowed");              throw new AutonomousRobotException(TAG, "Command EXECUTE_TARGET_ZONE_STEPS not allowed");

        } catch (ParserConfigurationException pex) {
            throw new AutonomousRobotException(TAG, "DOM parser Exception " + pex.getMessage());
        } catch (SAXException sx) {
            throw new AutonomousRobotException(TAG, "SAX Exception " + sx.getMessage());
        }
        //catch (XPathExpressionException xex) {
        //    throw new AutonomousRobotException(TAG, "XPath Expression Exception " + xex.getMessage());
        //}
        catch (IOException iex) {
            throw new AutonomousRobotException(TAG, "IOException " + iex.getMessage());
        }
    }

    // Get the target zone commands associated with an OpMode.
    // The key of the return map is the target zone.
    public HashMap<RobotConstantsUltimateGoal.TargetZone, List<RobotActionXML.CommandXML>> getTargetZoneCommands(RobotConstantsUltimateGoal.OpMode pOpMode) throws XPathExpressionException {
        HashMap<RobotConstantsUltimateGoal.TargetZone, List<RobotActionXML.CommandXML>> targetZoneCommands = new HashMap<>();
        List<RobotActionXML.CommandXML> commands;

        // Find an OpMode element with an "id" attribute equal to the pOpMode parameter.
        // <OpMode id="BLUE_INSIDE">
        String opModePath = "/TargetZones/OpMode[@id=" + "'" + pOpMode.toString() + "']";
        Node opModeNode = (Node) xpath.evaluate(opModePath, document, XPathConstants.NODE);
        if (opModeNode == null)
            throw new AutonomousRobotException(TAG, "Missing OpMode " + pOpMode);

        RobotLogCommon.d(TAG, "Processing target zone xml for OpMode " + pOpMode);

        // The OpMode element should have one child: <target_zones>
        //?? I want to use XPath expressions that are relative to the context node,
        // which I think is the OpMode node I've just selected. But no XPath expression
        // so far gets the child node opmode_target_zones.
        // A search string of child::opmode_target_zones or just opmode_target_zones returns null.
        // A search string of ".//opmode_target_zones"
        // returns a starting_position node from somewhere else in the document.
        // Node spNode = (Node) xpath.evaluate(exprString, document, XPathConstants.NODE);

        // Process the TARGET_ZONE A, B, C elements for the selected OpMode.
        String targetZonePath = opModePath + "/opmode_target_zones";

        // Target Zone A
        String exprString = targetZonePath + "/" + RobotConstantsUltimateGoal.TargetZone.TARGET_ZONE_A + "/*";
        NodeList targetZoneANodes = (NodeList) xpath.evaluate(exprString, document, XPathConstants.NODESET);
        if (targetZoneANodes == null)
            throw new AutonomousRobotException(TAG, "Missing TARGET_ZONE_A element");

        commands = collectCommands(targetZoneANodes);
        targetZoneCommands.put(RobotConstantsUltimateGoal.TargetZone.TARGET_ZONE_A, commands);

        // Target Zone B
        exprString = targetZonePath + "/" + RobotConstantsUltimateGoal.TargetZone.TARGET_ZONE_B + "/*";
        NodeList targetZoneBNodes = (NodeList) xpath.evaluate(exprString, document, XPathConstants.NODESET);
        if (targetZoneBNodes == null)
            throw new AutonomousRobotException(TAG, "Missing TARGET_ZONE_B element");

        commands = collectCommands(targetZoneBNodes);
        targetZoneCommands.put(RobotConstantsUltimateGoal.TargetZone.TARGET_ZONE_B, commands);

        // Target Zone C
        exprString = targetZonePath + "/" + RobotConstantsUltimateGoal.TargetZone.TARGET_ZONE_C + "/*";
        NodeList targetZoneCNodes = (NodeList) xpath.evaluate(exprString, document, XPathConstants.NODESET);
        if (targetZoneCNodes == null)
            throw new AutonomousRobotException(TAG, "Missing TARGET_ZONE_C element");

        commands = collectCommands(targetZoneCNodes);
        targetZoneCommands.put(RobotConstantsUltimateGoal.TargetZone.TARGET_ZONE_C, commands);

        return targetZoneCommands;
    }

    // Iterate through the children of the selected target zone node
    // and collect the elements.
    private List<RobotActionXML.CommandXML> collectCommands(NodeList pNodeList) {

        List<RobotActionXML.CommandXML> commands = new ArrayList<>();
        Node oneCommandNode;
        RobotActionXML.CommandXML commandXML;

        for (int i = 0; i < pNodeList.getLength(); i++) {
            oneCommandNode = pNodeList.item(i);

            if (oneCommandNode.getNodeType() != Node.ELEMENT_NODE)
                continue;

            commandXML = new RobotActionXML.CommandXML((Element) oneCommandNode);
            commands.add(commandXML);
        }

        return commands;
    }

}

