package org.firstinspires.ftc.ftcdevcommon;

import org.w3c.dom.Element;

// This is a convenience wrapper for a DOM Element to isolate
// XML access.
// Holds an XML element and its name.
    //   <WOBBLE_SERVO>
    //     <position>down</position>
    //   </WOBBLE_SERVO>
    // Use the companion class XPathAccess to get attributes,
    // child nodes, and text relative to this node.
    public class RobotXMLElement {

        private final Element robotXMLElement;

        // --------- CONSTRUCTORS ----------
        public RobotXMLElement(Element pRobotXMLElement) {
            robotXMLElement = pRobotXMLElement;
        }

        // --------- FUNCTIONS ----------
        public Element getRobotXMLElement() {
            return robotXMLElement;
        }

        public String getRobotXMLElementName() {
            return robotXMLElement.getTagName();
        }
    }