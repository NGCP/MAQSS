import QtQuick 2.7
import QtQuick.Controls 2.0
import QtQuick.Layouts 1.0
import QtPositioning 5.3
import QtLocation 5.4
import "Coordinates.js" as Coordinates
import "GPS.js" as GPS

Rectangle {
    id: mapContainer

    property int mapWidth
    property int mapHeight
    property var searchChunkContainer: []
    property var quadcopterIcons: []
    property real startLat: 35.308806
    property real startLon: -120.668194

    width: mapWidth
    height: mapHeight

    color: "gray"

    // TODO: Configure plugin to use google maps
    // [Initialize Plugin]
    Plugin {
        id: myPlugin
        name: "osm"
    }

    // TODO: Figure out how to cache map data
    Map {
        id: map
        anchors.fill: parent
        plugin: myPlugin;
        center {
            latitude: startLat
            longitude: startLon
        }

        gesture.enabled: true
        zoomLevel: 18.5
        activeMapType: map.supportedMapTypes[5]

        // TODO: Make icons load from local files
        // TODO: Make these not show up before a mission has been set
        // Two marker icons
        MapQuickItem {
            id: mark1
            sourceItem: Image {
                id: image1
                source: "http://icons.iconarchive.com/icons/paomedia/small-n-flat/128/map-marker-icon.png"
                scale: 0.1
                height: 50
                width: 50
            }
            anchorPoint.x: image1.width/2
            anchorPoint.y: image1.height
            coordinate: map.center
        }

        MapQuickItem {
            id: mark2
            sourceItem: Image {
                id: image2
                source: "http://icons.iconarchive.com/icons/paomedia/small-n-flat/128/map-marker-icon.png"
                scale: 0.1
                height: 50
                width: 50
            }
            anchorPoint.x: image2.width/2
            anchorPoint.y: image2.height
            coordinate: map.center
        }

        // Rectangle between marker icons
        MapPolygon {
            id: mapPolygon
            color: "transparent"
            opacity: 0.5
        }

        MouseArea {
            id: mapMouseArea
            anchors.fill: parent

            // wait for two clicks whenever captureButton is pressed
            onPressed: {
                if (controlPanelBox.captureState) {

                    if (pointsCaptured === 0 ) {
                        mark1.coordinate = map.toCoordinate(Qt.point(mouse.x, mouse.y))
                        mapPolygon.path = []
                    }
                    else if (pointsCaptured === 1) {
                        mark2.coordinate = map.toCoordinate(Qt.point(mouse.x, mouse.y))
                    }
                    pointsCaptured += 1

                    if (pointsCaptured >= 2) {

                        // draw polygon
                        mapContainer.update()

                        currentMsg = "Marker Distance: " + GPS.distance(mark1.coordinate,mark2.coordinate) + ", Marker Bearing: " + GPS.bearing(mark1.coordinate,mark2.coordinate)
                        messageBox.write(currentMsg)

                        controlPanelBox.captureState = false
                        controlPanelBox.state = "Waiting"
                        pointsCaptured = 0
                    }
                }

                else {
                    console.log("Point Capture Not Enabled")
                    var pt = map.toCoordinate(Qt.point(mouse.x, mouse.y));
                    console.log(pt.latitude, pt.longitude, pt.altitude)
                }
            }
        }
    }

    function update() {

        var polygonCoord = Coordinates.calculateCoords(mark1.coordinate, mark2.coordinate, mainPage.field_angle)
        var i1,j1
        var component
        var iconComponent
        var tmpCoord

        // Update main search Area
        mapPolygon.path = []
        for (i1 = 0; i1 < 5; i1++) {

            // add corner of search area rectangleS
            searchAreaCoords[i1] = QtPositioning.coordinate(polygonCoord[i1][0], polygonCoord[i1][1], polygonCoord[i1][2])
            mapPolygon.addCoordinate(searchAreaCoords[i1])
        }

        // divide search area and create searchChunk
        searchChunkCoords = Coordinates.divideSearchArea(searchAreaCoords,mainPage.field_angle, nQuickSearch)
        component = Qt.createComponent("SearchChunk.qml")

        // clear old search chunks
        if (searchChunkContainer.length > 0) {
            for (i1 = 0; i1 < searchChunkContainer.length; i1++) {
                searchChunkContainer[i1].destroy()
            }
        }

        // TODO: Implement reuse of MapPolygons instead of destruction (reduce computation time)
        for (i1 = 0; i1< nQuickSearch; i1++) {
            searchChunkContainer[i1] =  component.createObject(map)

            // warn object creation error
            if (searchChunkContainer[i1] === null) {
                console.log("Error Creating SearchChunk Object")
            }

            // Add corners of polygon for each Search Chunk
            searchChunkContainer[i1].path = []
            for (j1 = 0; j1 < 5; j1++) {
                searchChunkContainer[i1].addCoordinate(QtPositioning.coordinate(searchChunkCoords[i1][j1][0], searchChunkCoords[i1][j1][1], searchChunkCoords[i1][j1][2]))
            }

            // Set Search Chunk color
            searchChunkContainer[i1].color = searchChunkContainer[i1].availableColors[i1 % searchChunkContainer[i1].nColors]
            map.addMapItem(searchChunkContainer[i1])

            // update searchChunk messages
        }

        // Display message to indicate new Search Area Calculated
        currentMsg = "Search area added with " + nQuickSearch + " search chunks at " + field_angle + "° heading" +"\n\tCorners at: " + searchAreaCoords[0] + ", " + searchAreaCoords[2]
        messageBox.write(currentMsg)

        iconComponent = Qt.createComponent("QuadcopterIcon.qml")
        for (i1 = 0; i1 < quadcopters.length; i1++ ) {
//            tt = GPS.midPoint(searchChunkCoords[0][0],searchChunkCoords[0][2])
//            console.log(searchAreaCoords)
//            tmpCoord = QtPositioning.coordinate(tt[0],tt[1],tt[2])
            vehicleCoords[i1] = [quadcopters[i1].coordLLA[0], quadcopters[i1].coordLLA[1], quadcopters[i1].coordLLA[2]]

            // render the quadcopter icon at the starting coord
            tmpCoord = QtPositioning.coordinate(quadcopters[i1].coordLLA[0], quadcopters[i1].coordLLA[1])

            // if icon doesnt exist, create
            if (quadcopterIcons.length <= i1) {
                quadcopterIcons[i1] = iconComponent.createObject(map)
            }
            quadcopterIcons[i1].coordinate = tmpCoord
            quadcopterIcons[i1].iconColor = quadcopterIcons[i1].availableColors[quadcopters[i1].idNumber]
            map.addMapItem(quadcopterIcons[i1])
        }
    }
}

