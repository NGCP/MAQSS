function quaternionRotate(R, p, transpose) {

    // R is a 3 element array 3x1 arrays
    // p is a 3x1 array
    // transpose if a bool specifying if R should be transposed
    // return the result of R * p if R was a 3x3 matrix and p was a 3x1 matrix

    var mat = new Array

    if (transpose) {
        mat = [R[0][0]*p[0] + R[1][0]*p[1] + R[2][0]*p[2], R[0][1]*p[0] + R[1][1]*p[1] + R[2][1]*p[2], R[0][2]*p[0] + R[1][2]*p[1] + R[2][2]*p[2]]
    }
    else {
        mat = [R[0][0]*p[0] + R[0][1]*p[1] + R[0][2]*p[2], R[1][0]*p[0] + R[1][1]*p[1] + R[1][2]*p[2], R[2][0]*p[0] + R[2][1]*p[1] + R[2][2]*p[2]]
    }

    return mat
}

function quaternionRotationMatrix(vector, angle) {
    // vector is a 3D (3 element array) vector to rotate around (does not have to be unit vector)
    // angle is the angle to be rotated around vector (radians)
    // returns a quaternion Rotation matrix, R to perform the rotation (3, 3x1 arrays)

    var c = Math.cos(angle)
    var s = Math.sin(angle)
    var c1 = (1 - c)
    var u = findUnitVector(vector)
    var R = new Array
    var ndx

    for (ndx = 0; ndx < 3; ndx++){
        R[ndx] = new Array
    }

    R[0] = [c + Math.pow(u[0],2)*c1, u[0]*u[1]*c1 - u[2]*s, u[0]*u[2]*c1 + u[1]*s]
    R[1] = [u[1]*u[0]*c1 + u[2]*s, c + Math.pow(u[1],2)*c1, u[1]*u[2]*c1 - u[0]*s]
    R[2] = [u[2]*u[0]*c1 - u[1]*s, u[2]*u[1]*c1 + u[0]*s, c + Math.pow(u[2],2)*c1]

    return R
}

function findUnitVector(vector) {
    // vector is a nx1 dimension array
    // returns the unit vector of vector (norm = 1)
    // NOTE: This function uses the euclidean norm

    var unitVector = new Array
    var norm = findNorm(vector)
    var ndx

    for (ndx = 0; ndx < vector.length; ndx++) {
        unitVector[ndx] = 1/norm * vector[ndx]
    }

    return unitVector
}

function findNorm(vector) {
    // vector is a nx1 sized array
    // returns the euclidean norm of vector

    var ndx
    var norm = 0
    for (ndx = 0; ndx < vector.length; ndx++ ){

        norm = Math.pow(vector[ndx],2) + norm
    }
    return Math.sqrt(norm)
}


function xDCM(coordECEF, theta, transpose) {
    // if transpose is true, perform a multiplication with transpose
    // theta is in radians

    var rotatedECEF = [0,0,0]
    var x = coordECEF[0]
    var y = coordECEF[1]
    var z = coordECEF[2]
    if (transpose){

        rotatedECEF[0] = x
        rotatedECEF[1] = y * Math.cos(theta) - z * Math.sin(theta)
        rotatedECEF[2] = z * Math.cos(theta) + y * Math.sin(theta)
    }
    else {

        rotatedECEF[0] = x
        rotatedECEF[1] = y * Math.cos(theta) + z * Math.sin(theta)
        rotatedECEF[2] = z * Math.cos(theta) - y * Math.sin(theta)

    }
    return rotatedECEF

}

function xECEFtoLLA( coordECEF) {

    //    WGS84 Parameters
    var a = 6378137
    var b = 6356752.31424518
    var a2 = Math.pow(a,2)
    //                        var f = 1/298.257223563
    var b2 = Math.pow(b,2)
    var e = Math.sqrt((a2 - b2)/a2)
    var e2 = Math.pow(e,2)
    //                        var ep = Math.sqrt((a2 - b2)/b2)

    // function parameters
    var tolerance = .00000000001
    var X = coordECEF[0]
    var Y = coordECEF[1]
    var Z = coordECEF[2]

    var lam = Math.atan2(Y,X)
    var p = Math.sqrt(Math.pow(X,2) + Math.pow(Y,2))

    // Perform iterative loop
    var phi = Math.atan2(Z, p * (1 - e2))
    var h = 0
    var N = 0

    var phi_1 = -1
    var h_1 = -1
    var counter = 0

    while (Math.abs(phi - phi_1) > tolerance && Math.abs(h - h_1) > tolerance) {

        if (counter > 0) {
            phi = phi_1
            h = h_1
        }

        N = a / (Math.sqrt(1 - e2 * Math.pow(Math.sin(phi),2)))
        h_1 = p / Math.cos(phi) - N
        phi_1 = Math.atan2(Z, p * (1 - e2 * (N / (N + h_1))))

        // prevent infinite loop (should converge within 3-5 iterations)
        counter++
        if (counter > 1000) {
            console.log("Iterative loop not converged after 1000 runs. Breaking")
            break
        }
    }

    return [phi_1 * 180/Math.PI, lam * 180/Math.PI, h_1]
}

function xLLAtoECEF(coordLLA) {

    //    WGS84 Parameters
    var a = 6378137
    var b = 6356752.31424518
    var f = 1/298.257223563
    var a2 = Math.pow(a,2)
    var b2 = Math.pow(b,2)
    var e = Math.sqrt((a2 - b2)/a2)
    var e2 = Math.pow(e,2)
    var ep = Math.sqrt((a2 - b2)/b2)

    var phi = coordLLA[0]
    var lam = coordLLA[1]
    var h = coordLLA[2]

    // Convert to radians
    phi = phi * Math.PI/180
    lam = lam * Math.PI/180

    var N = a / Math.sqrt(1 - e2 * Math.pow(Math.sin(phi),2))
    var X = (N + h) * Math.cos(phi) * Math.cos(lam)
    var Y = (N + h) * Math.cos(phi) * Math.sin(lam)
    var Z = ((b2/a2) * N + h) * Math.sin(phi)

    return [X,Y,Z]

}

function xRotatedLLAtoLLA(rotatedCoordLLA, R) {
    // rotatedCoordLLA is variable length array. Each element is a 5 element array with each of those elements being [lat, lon, alt]
    // theta is the angle in radians of the rotated coordinate frame
    // R is the quaternion rotation matrix

    var len = rotatedCoordLLA.length
    var i1, j1
    var tmp = new Array

    for (i1 = 0; i1 < len; i1++ ){

        tmp[i1] = new Array
        for (j1 = 0; j1 < 5; j1++) {

            tmp[i1][j1] = new Array
            // boxCoords in rotated ECEF
            tmp[i1][j1] = xLLAtoECEF(rotatedCoordLLA[i1][j1])

            // boxCoords in normal ECEF
            tmp[i1][j1] = quaternionRotate(R, tmp[i1][j1], true)

            // boxCoords in normal LLA (return)
            tmp[i1][j1] = xECEFtoLLA(tmp[i1][j1])

        }
    }
    return tmp

}



