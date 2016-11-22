.import "Transforms.js" as Transforms
.import "GPS.js" as GPS
.import "Utils.js" as Utils

// TODO: Make CostFunctions enum a constant
var CostFunctions = {
    MIN_MAX_DIST : "MIN_MAX_DIST",
    MIN_TOTAL_DIST : "MIN_TOTAL_DIST",
    MIN_MEAN_DIST : "MIN_MEAN_DIST",
};

function findBestCombo(midPointDistances, combos, costFun) {
    /* Function to return the best vehicle -> searchChunk allocation given a specified cost function
        Input:
            midPointDistances - array of distances from each quadcopter to each searchChunk midPoint
            combos  - array of vehicle to searchChunk combinations
            costFun - enum value specifying which cost function to use (from CostFunctions enum)
        Output:
            Returns an int corresponding to the best combination in combos
    */

    // find best matches with specified cost function
    var best;
    var i1, j1;
    var maxD = new Array;
    var average = new Array;
    var totalDist = new Array;
    var travelDistances = new Array;
    for (i1 = 0; i1 < combos.length; i1++ ) {
        travelDistances[i1] = new Array;
        for (j1 = 0; j1 < midPointDistances[0].length; j1++ ) {
            travelDistances[i1][j1] = midPointDistances[j1][combos[i1][j1]];
        }
        maxD[i1] = Utils.max(travelDistances[i1])[0];
        totalDist[i1] = travelDistances[i1].reduce(function(a,b) {return a + b;} , 0);
        average[i1] = totalDist[i1]/travelDistances[i1].length;
    }

    // store index of best combo according to cost function
    if (costFun === CostFunctions.MIN_MAX_DIST) best = Utils.min(maxD)[1];
    else if (costFun === CostFunctions.MIN_TOTAL_DIST) best = Utils.min(totalDist)[1];
    else if (costFun === CostFunction.MIN_MEAN_DIST) best = Utils.min(average)[1];
    else throw "Unsupported cost function type"

    return combos[best];
}

function allocateSearchChunks(searchChunks, vehicleCoords, fieldHeading) {
    /* Function to allocate searchChunks to available quadcopters
         Input:
          searchChunks  - searchChunkCoords array from mainPage. Contains array of n elements equal to
                            number of searchChunks. Each element of this array contains a 5 element array
                            of [lat lon alt]
          vehicleCoords - Array with length equal to number of searchChunks and vehicles. Each element of
                            array contains [lat lon alt] corresponding to one Quadcopter vehicle location
        Output:
            Returns an array of message strings to be transmitted to quadcopters via C++ API. Length of
            array will equal number of searchChunks and number of vehicles
    */

    var n = searchChunks.length; // number of searchChunks
    var i1, j1;
    var cornerChoice
    var uniqueElements
    var msnString = new Array;
    var midPoints = new Array;
    var choice = new Array;

    /* midPointDistances array has the form:
      [   [chunk1 -> QuadA, chunk1 -> QuadB, chunk1 -> QuadC, ...],
          [chunk2 -> QuadA, chunk2 -> QuadB                   ...],
          [chunk3 -> QuadA                                    ...],
          ...
      ]
    */
    var midPointDistances = new Array; // nxn array of distances between vehicles and searchChunk midpoints

    // TODO: If searchChunks/vehicleCoords is only 1 element long, skip to bottom
    for (i1 = 0; i1 < n; i1++) {
        // find midpoint of each searchChunk
        midPoints[i1] = GPS.midPoint(searchChunks[i1][0], searchChunks[i1][2]);
        midPointDistances[i1] = new Array
        for (j1 = 0; j1 < n; j1++) midPointDistances[i1][j1] = GPS.arrDistance(vehicleCoords[j1], midPoints[i1]);

        // store initial selection of choice
        choice[i1] = Utils.min(midPointDistances[i1])[1];
    }

    // determine if all searchChunks were matched to vehicles uniquely first time around (lucky!)
    uniqueElements = choice.filter( Utils.onlyUnique );

    // if everything doesnt work out first time
    if (uniqueElements.length !== n) {

        // create permutations and perform n! calculations
        // TODO: Implement better algorithm that isnt O(n!)
        var combos = new Array;
        var maxD = new Array;
        var arr = Utils.range(n)
        Utils.heapsPermute(arr , combos);
        console.log(combos);
        choice = findBestCombo(midPointDistances, combos, CostFunctions.MIN_MAX_DIST);
    }

    // proceed to finding closest corner
    cornerChoice = allocateCorners(searchChunks, vehicleCoords, choice);
    msnString = writeMsnString(searchChunks, vehicleCoords, choice, cornerChoice, fieldHeading);

    return msnString
}

function writeMsnString(searchChunks, vehicleCoords, choices, cornerChoices, fieldHeading) {
    /* Function to generate an array of strings to send to quadcopter vehicles
        Messages will be in the form
        ["Q<QuadID>, P<Lat> <Lon> <Alt>, H<Heading Degs>, D<Distance Meters>"]
        Example:
            ["Q0,P35.123456 120.123456 0.0000,H1.23456,F123.4,D100.4567",
             "Q1,P35.567889 120.765432 0.0000,H89.7654,F456.7,D200.4567"]

         Input:
              searchChunk - searchChunkCoords array from mainPage. Contains array of n elements equal to
                          number of searchChunks. Each element of this array contains a 5 element array
                          of [lat lon alt]
              vehicleCoords - Array with length equal to number of searchChunks and vehicles. Each element of
                          array contains [lat lon alt] corresponding to one Quadcopter vehicle location
              choices     - array of ints the same length as searchChunk and vehicleCoords. Each element is an
                          index corresponding to the vehicle which matches the search chunk at that index.
                          (ex. choices = [1 0] means chunk[0]-> quad[1], chunk[1] -> quad[0]
              cornerChoices - array with length equal to all other inputs. Each element is a position corresponding
                          to the corner that searchChunk should be started on. Position 0 corresponds to the top right
                          corner and moves counter clockwise
              fieldHeading  - heading of the field/search area in degrees
         Output:
              returns an array of strings to be sent off the quadcopters
    */
    var i1;
    var distance;
    var msnString = new Array;
    var headings = new Array;
    var startLocs = new Array;

    // TODO: Include heading of field in msg string also
    // calculate distances before hand, should be the same for all vehicles (diagonal distance)
    distance = GPS.arrDistance(searchChunks[0][0], searchChunks[0][2]);

    for (i1 = 0; i1 < vehicleCoords.length; i1 ++ ) {
        startLocs[i1] = searchChunks[i1][cornerChoices[i1]];

        // calculate the heading for searchChunk
        headings[i1] = cornerChoices[i1] > 2 ? GPS.arrBearing(searchChunks[i1][cornerChoices[i1]], searchChunks[i1][cornerChoices[i1] - 2]) :
                                               + GPS.arrBearing(searchChunks[i1][cornerChoices[i1]], searchChunks[i1][cornerChoices[i1] + 2]);

        // Generate message string following format in heading "NEWMSG,MSN,Q1,P35.123456 120.123456 0.0000,H1.23456,F139.7,D100.4567"
        msnString[i1] = "NEWMSG" + ",MSN"
                + ",Q" + mainPage.quadcopters[choices[i1]].idNumber
                + ",P" + String(startLocs[i1][0]) + " " + String(startLocs[i1][1]) + " " + String(startLocs[i1][2])
                + ",H" + String(headings[i1])
                + ",F" + String(fieldHeading)
                + ",D" + String(distance);
    }
    return msnString;
}

function allocateCorners(searchChunks, vehicleCoords, choices) {
    /* Function to choose the closest starting corner for each vehicle after searchChunks have been allocated
         Input:
              searchChunk - searchChunkCoords array from mainPage. Contains array of n elements equal to
                          number of searchChunks. Each element of this array contains a 5 element array
                          of [lat lon alt]
              vehicleCoords - Array with length equal to number of searchChunks and vehicles. Each element of
                          array contains [lat lon alt] corresponding to one Quadcopter vehicle location
              choices     - array of ints the same length as searchChunk and vehicleCoords. Each element is an
                          index corresponding to the vehicle which matches the search chunk at that index.
                          (ex. choices = [1 0] means chunk[0]-> quad[1], chunk[1] -> quad[0]
         Output:
              returns array of ints equal to length of input arrays. Each element corresponds to a corner
              position for that searchChunk to be started at. position 0 is top right corner and moves
              counter clockwise
    */

    if (searchChunks.length !== choices.length) throw "Number choices not equal to number searchChunks/vehicles";

    var i1, j1;
    var distances = new Array;
    var cornerChoice = new Array;

    for (i1 = 0; i1 < choices.length; i1++ ) {
        for (j1 = 0; j1 < searchChunks[i1].length - 1; j1++) { //should be j1 < 5
            distances[j1] = GPS.arrDistance(searchChunks[i1][j1], vehicleCoords[choices[i1]]);
        }
        cornerChoice[i1] = Utils.min(distances)[1];
    }

    return cornerChoice;
}

function calculateCoords(topLeft, bottomRight, angle) {
    // Calculates the GPS coordinates for a rectangle at angle degrees with corners at topLeft and bottomRight

    //    var mat = new Matrix.Matrix(3,3)

    // topLeft and bottomRight are coordinate types
    var theta = angle * Math.PI/180.0
    var ndx = 0
    var coord1 = [topLeft.latitude, topLeft.longitude, topLeft.altitude]
    var coord2 = [bottomRight.latitude, bottomRight.longitude, topLeft.altitude]
    var midPoint = GPS.midPoint(coord1,coord2)
    var coords = [coord1, coord2]
    var R
    var rotatedECEF = new Array
    var rotatedLLA = new Array
    var boxCoords = new Array

    midPoint = Transforms.xLLAtoECEF(midPoint)
    R = Transforms.quaternionRotationMatrix(midPoint, theta)

    for (ndx = 0; ndx < coords.length; ndx++) {
        coords[ndx] = Transforms.xLLAtoECEF(coords[ndx])
        rotatedECEF[ndx] = Transforms.quaternionRotate(R, coords[ndx], false)
        rotatedLLA[ndx] = Transforms.xECEFtoLLA(rotatedECEF[ndx])
    }

    // boxCoords in rotated LLA
    boxCoords = drawRectangle([rotatedLLA[0][0], rotatedLLA[1][0],rotatedLLA[0][1],rotatedLLA[1][1], rotatedLLA[0][2]])

    for (ndx = 0; ndx < boxCoords.length; ndx++ ) {
        // boxCoords in rotated ECEF
        boxCoords[ndx] = Transforms.xLLAtoECEF(boxCoords[ndx])

        // boxCoords in normal ECEF
        boxCoords[ndx] = Transforms.quaternionRotate(R, boxCoords[ndx], true)

        // boxCoords in normal LLA (return)
        boxCoords[ndx] = Transforms.xECEFtoLLA(boxCoords[ndx])
    }

    // returns a 5 element array of 3 elements (5 x 3 array)
    return boxCoords
}

function divideSearchArea(searchArea, angle, nVehicles) {
    /*
  Inputs
    searchArea - 5 element array of QtPositioning coordinates specifying a closed rectangle
    angle - heading angle of rectangle in degrees
    nVehicles - number of quick search vehicles to divide the search area into
  */

    // transform coordinate frame
    var theta = angle * Math.PI/180.0
    var ndx = 0
    var midPoint = GPS.midPoint([searchArea[0].latitude, searchArea[0].longitude, searchArea[0].altitude],[searchArea[2].latitude, searchArea[2].longitude, searchArea[2].altitude])
    var R
    var searchAreaLLA = new Array
    var searchAreaECEF = new Array
    var rotatedAreaECEF = new Array
    var rotatedAreaLLA = new Array
    var splitRotatedAreaLLA = new Array
    var splitAreaLLA = new Array

    var length, width

    midPoint = Transforms.xLLAtoECEF(midPoint)
    R = Transforms.quaternionRotationMatrix(midPoint, theta)

    for (ndx = 0; ndx < 5; ndx++) {
        searchAreaLLA[ndx] = [searchArea[ndx].latitude, searchArea[ndx].longitude, searchArea[ndx].altitude]

        searchAreaECEF[ndx] = Transforms.xLLAtoECEF(searchAreaLLA[ndx])
        rotatedAreaECEF[ndx] = Transforms.quaternionRotate(R, searchAreaECEF[ndx], false)
        rotatedAreaLLA[ndx] = Transforms.xECEFtoLLA(rotatedAreaECEF[ndx])
    }

    // find larger dimension between length and width
    width = GPS.distance(searchArea[1], searchArea[2])
    length = GPS.distance(searchArea[0], searchArea[1])

    // divide area
    if (length >= width) {

        splitRotatedAreaLLA = splitBox(rotatedAreaLLA, nVehicles, 1)
    }
    else {
        // divide width
        splitRotatedAreaLLA = splitBox(rotatedAreaLLA, nVehicles, 0)

    }

    // create input array for vehicles
    splitAreaLLA = Transforms.xRotatedLLAtoLLA(splitRotatedAreaLLA, R)

    // return array
    return splitAreaLLA

}

function drawRectangle( corners ) {
    // 5 element array containing two corners of the box [lat1, lat2, lon1, lon2, avgAlt]
    var x1 = corners[0]
    var x2 = corners[1]
    var y1 = corners[2]
    var y2 = corners[3]
    var alt = corners[4]

    return [[x1, y1,alt], [x2, y1,alt], [x2, y2,alt], [x1, y2,alt], [x1, y1,alt]]
}

function splitBox(boxCorners, nParts, method) {
    // boxCorners is a 5 element array with each element containing [lat, lon, alt] for one of the corners of a rectangle
    // nParts is an int specifying how many sections to break the box into
    // assumes altitude is contant throughout
    // method - specify division of width(0) or length(1)

    // store altitude and work in only 2 dimensions
    var alt = boxCorners[0][2]

    var x0 = boxCorners[0][0]
    var y0 = boxCorners[2][1]
    var x1 = boxCorners[1][0]
    var y1 = boxCorners[0][1]

    var dx = x1 - x0;
    var dy = y1 - y0;

    var ndx = 0
    var i1 = 0
    var xp, xp_1
    var yp, yp_1

    var splitBox = new Array

    for  (ndx = 0; ndx < nParts; ndx++ ) {

        splitBox[ndx] = new Array
        for (i1 = 0; i1 < 5; i1++ ) {
            splitBox[ndx][i1] = new Array
        }


        if (method) {
            xp = x0 + ndx * dx/nParts
            xp_1 = x0 + (ndx + 1) * dx / nParts
            splitBox[ndx][0] = [xp, y0, alt]
            splitBox[ndx][1] = [xp, y1, alt]
            splitBox[ndx][2] = [xp_1, y1, alt]
            splitBox[ndx][3] = [xp_1, y0, alt]
            splitBox[ndx][4] = splitBox[ndx][0]
        }
        else {
            yp = y0 + ndx * dy/nParts
            yp_1 = y0 + (ndx + 1) * dy/nParts
            splitBox[ndx][0] = [x0, yp, alt]
            splitBox[ndx][1] = [x1, yp, alt]
            splitBox[ndx][2] = [x1, yp_1, alt]
            splitBox[ndx][3] = [x0, yp_1, alt]
            splitBox[ndx][4] = splitBox[ndx][0]

        }
    }

    return splitBox
}
