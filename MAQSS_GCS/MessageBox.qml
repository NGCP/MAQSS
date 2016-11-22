import QtQuick 2.0
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.0

Rectangle {
    id: rectangle
    property int msgPadding: padding/2
    property string displayText: "Message Box"
    property int fontSize: 10
    property string msgCache
    property string prevMsg
    anchors {
        left: parent.left
        leftMargin: msgPadding
        bottom: parent.bottom
        bottomMargin: msgPadding
    }
    width: parent.width - 2 * msgPadding
    height: parent.height * 0.25 - 2 * msgPadding
    color: "transparent"

    function write(msg) {
        // Prevent display of duplicate messages
        if (msg !== prevMsg) {
            msgCache = msg + "\n" + msgCache
            displayText = msgCache
        }
        prevMsg = msg
    }

    Flickable {
        id: flickable
        anchors.fill: parent
        TextArea.flickable: TextArea {
            id: messageBoxText
            anchors.fill: parent
            color: "black"
            text: rectangle.displayText
            font {
                pointSize: fontSize
            }
            readOnly: true
            activeFocusOnPress: false
            selectByMouse: false
            verticalAlignment: TextField.AlignTop
        }

        ScrollBar.vertical: ScrollBar {
            position: 1.0
        }
    }
}
