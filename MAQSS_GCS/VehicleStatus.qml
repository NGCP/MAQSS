import QtQuick 2.0
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.0

Rectangle {
    property int vehicleStatusPadding: 10

    property string title: "Vehicle Monitor"
    property bool titleBold: true
    property int titleFontSize: 13

    property string label1: "Vehicle"
    property string label2: "Status"
    property bool labelBold: true
    property int labelFontSize: 12

    property int vehicleFontSize: 12

    signal updateStatus(bool update)

    id: vehicleStatusBox
    TextField {
        id: vehicleStatusTitle
        anchors {
            top: vehicleStatusBox.top
            left: vehicleStatusBox.left
        }

        background: Rectangle {
            implicitWidth: vehicleStatusBox.width
            implicitHeight: vehicleStatusBox.height/6
            color: "transparent"
            border.color: "transparent"
        }
        text: title

        font {
            bold: titleBold
            pointSize: titleFontSize
        }

        readOnly: true
        selectByMouse: false
    }

    onUpdateStatus: {
        var i1 = 0

        // create new ListElements if they dont exist
        while (vehicleModel.count < quadcopters.length) {
            vehicleModel.append({name: quadcopters[i1].name, status: quadcopters[i1].status})
        }

        // update vehicle status display
        for (i1 = 0; i1< quadcopters.length; i1++ ) {
            vehicleModel.set(i1, {name: quadcopters[i1].name, status: quadcopters[i1].status})
        }
    }

    TextField {
        id: vehicleStatusLabels
        anchors {
            top: vehicleStatusTitle.bottom
            left: vehicleStatusBox.left
        }

        background: Rectangle {
            implicitWidth: vehicleStatusBox.width
            implicitHeight: vehicleStatusBox.height/8
            color: "transparent"
            border.color: "transparent"
        }

        text: label1 + " \t\t" + label2
        font {
            bold: labelBold
            pointSize: labelFontSize
        }
        readOnly: true
        selectByMouse: false

    }

    ListModel {
        id: vehicleModel
    }

    Component {
        id: vehicleComponent
        Text {
            text: name + " \t\t" + status
            font {
                pointSize: vehicleFontSize
            }
        }
    }

    ListView {
        id: vehicleStatusList
        anchors {
            top: vehicleStatusLabels.bottom
            left: vehicleStatusLabels.left
            leftMargin: vehicleStatusPadding
            bottom: vehicleStatusBox.bottom
        }
        model: vehicleModel
        delegate: vehicleComponent
        spacing: 4
    }
}
