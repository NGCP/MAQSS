import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.0
import QtPositioning 5.3
import QtLocation 5.4

MapPolygon {
    property var availableColors: ["dark blue", "green", "red", "blue", "violet","yellow","black","white"]
    property int nColors: availableColors.length

    id: searchChunkArea
    opacity: 0.33

}

