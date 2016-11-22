import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.0
import XbeeInterface 1.0
import "Utils.js" as Utils

/* Main Page Object
  A single MainPage object is created in the ApplicationWindow


  Creates and Contains Objects:
    -MapContainer.qml
    -VehicleStatus.qml
    -VehicleStatus.qml
    -MessageBox.qml

  Instantiates Singleton Class XbeeInterface to perform async read/write

  */
Item {
    // TODO: Add a caution "Demo" button (makes em dance)
    // TODO: Add area to input baudrate and deviceID
    // TODO: Add timeout functionality to set vehicle status as offline
    property int padding: 25
    property int pointsCaptured: 0  // DO NOT DELETE (Stores UI capture information)
    property int nQuickSearch: 0    // stores number of vehicles in the QuickSearch (Quick Scan) Role
    property int nDetailedSearch: 0 // stores number of vehicles in the DetailedSearch role
    property int baudRate: 57600    // serial interface baudRate

    property string currentMsg      // current message to be written to vehicles


    property bool capture: false    // stores if UI is in capture mode
    property bool missionSet: false // store if mission has been set by user

    property real field_angle: 139.7
    property var searchAreaCoords: []
    property var searchChunkCoords: [] // javascript array of nQuickSearch elements. Each element is a 5 element array of [lat lon alt] (3D array)
    property var searchChunkMessages: [] // string specifying search chunk
    property var vehicleCoords: [] // array which stores the locations of each active vehicle

    property var quadcopters: [] // array which stores Quadcopter.qml components

    signal startSignal(var object) // signal to indicate the start button has been toggled

    id: mainPage
    width: parent.width
    height: parent.height

    Item {
        id: testRoot
        Component.onCompleted: {
            XbeeInterface.newMsg.connect(handleNewMsg)
        }
    }

    Rectangle {
        id: background
        anchors.centerIn: parent
        width: parent.width - parent.padding
        height: parent.height - parent.padding
        color: "transparent"

        // Container for the Map
        MapContainer {
            id: mapContainer
            anchors {
                left: parent.left
                top: parent.top
                topMargin: 0
                leftMargin: 0
            }

            mapWidth: parent.width * 0.75
            mapHeight: parent.height * 0.75
        }

        // Container for Vehicle Monitor and Control Buttons
        Rectangle {
            id: statusContainer
            width: parent.width * 0.25

            height: mapContainer.height
            color: "white"
            anchors {
                top: parent.top
                left: mapContainer.right
                topMargin: 0
                leftMargin: 0
            }

            // Vehicle Monitor Box
            VehicleStatus {
                id: vehicleStatusBox
                anchors {
                    top: statusContainer.top
                    topMargin: padding
                    left: statusContainer.left
                    leftMargin: padding
                }
                height: statusContainer.height/2
                width: statusContainer.width - 2 * padding
                color: "transparent"
            }

            // Control Button
            ControlPanel {
                id: controlPanelBox
                anchors {
                    left: statusContainer.left
                    leftMargin: padding
                    bottom: statusContainer.bottom
                    bottomMargin: padding
                }
                color: "transparent"
                height: statusContainer.height/2
                width: statusContainer.width - 2 * padding

            }

            // TODO: Add heading tool

        }
        // Message box
        MessageBox {
            id: messageBox
            fontSize: 12
        }

    }

    // TODO: Move to one of the .js files
    function handleNewMsg(msg) {
        /* Function to handle a new msg received by async read from XbeeInterface

          This function will:
          1. Split the msg
          2. Check if a Quadcopter component exists for the given Vehicle ID
          3. Create a Quadcopter component or update an existing component
          4. Update the VehicleStatus object
          5. Update the MapContainer object and QuadcopterIcon location

          A msg will have the form below:
          Q0,P35.300151 -120.661846 104.636000,SOnline,R0

          QX - Vehicle ID
          PXX.XXXX -XXX.XXXX XXX.XXX - Vehicle GPS Location
          SXXXXXXX - Vehicle Status (Online, Started, Offline, etc.)
          RX - Vehicle Role (0->Quick Scan, 1->Detailed Search)
          */
        var component
        var tmp = new Array // arrays can have different data types
        var i1;

        // split the msg for parsing
        // TODO: Move this into a function
        msg = msg.split(",")
        var quadID = parseInt(msg[0][1])
        var quadLoc = (msg[1].substr(1)).split(" ")

        quadLoc = Utils.strToFloat(quadLoc)
        var quadStatus = msg[2]
        quadStatus = quadStatus.substr(1)
        var quadRole = msg[3][1]
        var nextNdx = quadcopters.length;
        var doneFlag = false

        // wait for Quadcopter.qml component to be created
        component = Qt.createComponent("Quadcopter.qml")
        console.log(component.errorString())
        while (component.status !== Component.Ready) {
        }

        // check if quadID already exists
        for (i1 = 0; i1 < quadcopters.length; i1++) {
            if (quadcopters[i1].idNumber === quadID) {
                quadcopters[i1].coordLLA = quadLoc
                quadcopters[i1].status = quadStatus
                quadcopters[i1].role = parseInt(quadRole)
                doneFlag = true // set flag to indicate frame has been processed
            }
        }

        // else, create a new Quadcopter object
        if (!doneFlag && component.status === Component.Ready) {
            quadcopters[nextNdx] = component.createObject(mapContainer, {"idNumber": quadID, "coordLLA": quadLoc})

            // warn object creation error
            if (quadcopters[nextNdx] === null) {
                console.log("Error Creating Quadcopter Object")
            }
            //                quadcopters[nextNdx].coordLLA = quadLoc
            quadcopters[nextNdx].status = quadStatus
            quadcopters[nextNdx].role = parseInt(quadRole)
            currentMsg = "Vehicle: " + quadcopters[nextNdx].name + " connected with status: " + quadcopters[nextNdx].status
            messageBox.write(currentMsg)

            console.log(quadRole, !quadRole, quadRole === 0, !quadcopters[nextNdx].role)
            if (!quadcopters[nextNdx].role) nQuickSearch++ // increment
            else nDetailedSearch++
        }

        // send vehicleStatusBox an updateStatus signal
        vehicleStatusBox.updateStatus(true)
        mapContainer.update()
    }
}


