// TODO: Implement coordAt() function
function distance(coord1, coord2) {
    // coord1/coord2 are QtPositioning.coordinate types
    // returns distance in meters

    var phi1 = coord1.latitude * Math.PI/180
    var phi2 = coord2.latitude * Math.PI/180
    var lam1 = coord1.longitude * Math.PI/180
    var lam2 = coord2.longitude * Math.PI/180
    var dphi = phi2 - phi1
    var dlam = lam2 - lam1
    var Rearth = 6371000 // radius of earth [m]

    var a, c
    a = Math.pow(Math.sin(dphi/2),2) + Math.cos(phi1) * Math.cos(phi2) * Math.pow(Math.sin(dlam/2),2)
    c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a))
    return Rearth * c // distance in meters
}

function arrDistance(coord1, coord2) {
    // coord1/coord2 are arrays of [lat lon alt]
    // returns distance in meters

    var phi1 = coord1[0] * Math.PI/180
    var phi2 = coord2[0] * Math.PI/180
    var lam1 = coord1[1] * Math.PI/180
    var lam2 = coord2[1] * Math.PI/180
    var dphi = phi2 - phi1
    var dlam = lam2 - lam1
    var Rearth = 6371000 // radius of earth [m]

    var a, c
    a = Math.pow(Math.sin(dphi/2),2) + Math.cos(phi1) * Math.cos(phi2) * Math.pow(Math.sin(dlam/2),2)
    c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a))
    return Rearth * c // distance in meters
}

function bearing(coord1,coord2) {
    // coord1/coord2 are QtPositioning.coordinate types
    // returns bearing in degrees

    var phi1 = coord1.latitude * Math.PI/180
    var phi2 = coord2.latitude * Math.PI/180
    var lam1 = coord1.longitude * Math.PI/180
    var lam2 = coord2.longitude * Math.PI/180
    //    var dphi = phi2 - phi1
    var dlam = lam2 - lam1
    var Rearth = 6371000 // radius of earth [m]
    var X,Y

    X = Math.cos(phi2) * Math.sin(dlam)
    Y = Math.cos(phi1) * Math.sin(phi2) - Math.sin(phi1) * Math.cos(phi2) * Math.cos(dlam)
    return Math.atan2(X,Y) * 180/Math.PI
}

function arrBearing(coord1,coord2) {
    // coord1/coord2 arrays of [lat lon alt]
    // returns bearing in degrees

    // TODO: combine arrBearing and bearing into a single function (type checking?)
    var phi1 = coord1[0] * Math.PI/180
    var phi2 = coord2[0] * Math.PI/180
    var lam1 = coord1[1] * Math.PI/180
    var lam2 = coord2[1] * Math.PI/180
    //    var dphi = phi2 - phi1
    var dlam = lam2 - lam1
    var Rearth = 6371000 // radius of earth [m]
    var X,Y

    X = Math.cos(phi2) * Math.sin(dlam)
    Y = Math.cos(phi1) * Math.sin(phi2) - Math.sin(phi1) * Math.cos(phi2) * Math.cos(dlam)
    return Math.atan2(X,Y) * 180/Math.PI
}

function coordAt(coord1, dist, heading) {
    // coord1 is a 3 element array [lat, lon, alt]
    // dist is the distance away [m]
    // heading is the heading angle [deg]
    // returns a 3 element array [lat, lon, alt] containing GPS coordinates, "dist" away in "heading" direction from coord1
}

function midPoint(coord1, coord2) {
    // coord1 and coord2 3 element arrays containing [lat, lon, alt]
    // returns a 3 element array which is the mid point between coord1 and coord2, [lat, lon, alt]

    var phi1 = coord1[0] * Math.PI/180
    var phi2 = coord2[0] * Math.PI/180
    var lam1 = coord1[1] * Math.PI/180
    var dlam = coord2[1] * Math.PI/180 - lam1
    var Bx = Math.cos(phi2) * Math.cos(dlam)
    var By = Math.cos(phi2) * Math.sin(dlam)
    var midPoint = new Array

    midPoint[0] = 180/Math.PI * Math.atan2(Math.sin(phi1) + Math.sin(phi2), Math.sqrt(Math.pow(Math.cos(phi1) + Bx,2) + Math.pow(By,2)))
    midPoint[1] = 180/Math.PI * (lam1 + Math.atan2(By, Math.cos(phi1) + Bx))
    midPoint[2] = (coord1[2] + coord2[2])/2

    return midPoint

}
