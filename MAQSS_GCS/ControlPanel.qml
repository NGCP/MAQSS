import QtQuick 2.0
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.0
import XbeeInterface 1.0
import "Transforms.js" as Transforms
import "Coordinates.js" as Coordinates

Rectangle {
    id: controlPanelBox

    property int controlPanelPadding: 25
    property int buttonHeight: controlPanelBox.height/5
    property int buttonWidth: controlPanelBox.width/2
    property bool captureState: false
    property var searchChunkMessages: []

    // States for capturing/waiting to change CaptureButton behavior and appearance
    states: [
        State {
            name: "Capturing"; when: captureButton.checked
            PropertyChanges { target: controlPanelBox; captureState: true}
            PropertyChanges { target: captureButton; text: qsTr("Capturing")}
        },
        State {
            name: "Waiting";
            PropertyChanges { target: controlPanelBox; captureState: false }
            PropertyChanges { target: captureButton; text: qsTr("Set Mission") }
            PropertyChanges { target: captureButton; checked: false }
        }
    ]

    // Text input for field_angle
    TextField {
        id: headingInput
        color: "black"
        anchors {
            bottom: controlPanelBox.bottom
            bottomMargin: controlPanelPadding
            left: controlPanelBox.left
            leftMargin: 0
        }
        height: buttonHeight
        width: buttonWidth
        placeholderText: qsTr("Enter Angle (deg)")
        onEditingFinished: {
            mainPage.field_angle = parseFloat(text)
            focus = false
            mapContainer.update()
        }
    }

    // Button to transition into mission capture mode
    Button {
        id: captureButton
        state: "Waiting"
        anchors {
            left: controlPanelBox.left
            leftMargin: 0
            bottom: headingInput.top
            bottomMargin: controlPanelPadding/2
        }
        width: buttonWidth
        height: buttonHeight
        text: qsTr("Set Mission")
        highlighted: false
        checkable: true
    }

    // Switch to toggle mission start (send search chunks off to vehicles)
    Switch {
        id: startSwitch
        width: buttonWidth  * 0.5
        height: buttonHeight
        scale: parent.width/parent.height * 1.5

        anchors.right: controlPanelBox.right
        anchors.rightMargin: padding
        anchors.verticalCenter: captureButton.verticalCenter

        states: State {
            name: "Started"; when: startSwitch.checked === true
            PropertyChanges { target: switchText; text: qsTr("Stop")}
        }
        Text {
            id: switchText
            anchors.bottom: parent.top
            anchors.horizontalCenter: parent.horizontalCenter
            text: qsTr("Start")
        }

        // Evaluate state when clicked
        onClicked: {
            console.log("Press")
            var ndx;

            // Start state, allocate search Chunks and send msg off to vehicles
            if (startSwitch.state === "Started") {

                captureButton.checkable = false
                console.log("Started")
                startSignal(mainPage)
                currentMsg = "Starting Mission -- Transmitting"
                messageBox.write(currentMsg)

                // run function to allocate quadcopters to closest point
                var msg = Coordinates.allocateSearchChunks(mainPage.searchChunkCoords, mainPage.vehicleCoords, mainPage.field_angle)
                // TODO: Write msg to Quads here
                for (ndx = 0; ndx < msg.length; ndx++) {
                    XbeeInterface.writeMsg(msg[ndx]);

                    // each element of the quadcopters array is a Quadcopter.qml object
                    // TODO: Remove double write once xbeeplus library is fixed (read works 100%)
                    XbeeInterface.writeMsg("NEWMSG,START,Q" + quadcopters[ndx].idNumber);
                    XbeeInterface.writeMsg("NEWMSG,START,Q" + quadcopters[ndx].idNumber);
                }
            }

            // stop state, send STOP signal to all vehicles
            else {
                captureButton.checkable = true
                currentMsg = "Halting Mission"
                messageBox.write(currentMsg)
                for (ndx = 0; ndx < quadcopters.length; ndx++) {
                    XbeeInterface.writeMsg("NEWMSG,STOP,Q" + quadcopters[ndx].idNumber);
                    XbeeInterface.writeMsg("NEWMSG,STOP,Q" + quadcopters[ndx].idNumber);
                }
            }
        }
    }
}
