import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.0

/* Main Application
  Creates a MainPage object and names the ApplicationWindow
  NOTE: Refer to MainPage.qml
  */

ApplicationWindow {
    visible: true
    width: 1280
    height: 720
    title: qsTr("Ground Control Station")

    MainPage {
        objectName: "mainPage"
        id: mainPage
        x: 0
        y: 0
    }
}
