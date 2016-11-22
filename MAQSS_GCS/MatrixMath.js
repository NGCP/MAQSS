/* Matrix math assuming arrays as matricies

  Assumes matrix format to be

        //col 1     //col 2     //col 3
  M = [ [1,         2,          3],  // row 1
        [4,         5,          6],  // row 2
        [7,         8,          9]]  // row 3
*/


function Matrix(rows, cols) {

    this.rows = rows
    this.cols = cols
    this.data = []

    this.allocate = function(){
    // allocates the size of the data array
        var rows = this.rows
        var cols = this.cols
        var ndx

        this.data = new Array(rows)

        for (ndx = 0; ndx < rows; ndx++) {
            this.data[ndx] = new Array(cols)
        }
    }

    this.fill = function(val) {
        // overwrites and fills the elements of the matrix with value

        var i1, j1
        for (i1 = 0; i1 < this.rows; i1++) {

            for (j1 = 0; j1 < this.cols; j1++ ) {
                this.data[i1][j1] = val;
            }
        }

    }

    this.allocate()
    this.fill(0)
}


function transpose(matrix) {


}

function multiply(mat1, mat2) {

    // Multiplies two Matrix type objects. Will multiply mat1 * mat2

    // check sizes
    if (mat1.cols !== mat2.rows) {
        console.log("Incorrect Matrix Sizes. Mat1: ", mat1.rows, "x", mat1.cols, ", Mat2: ", mat2.rows, "x", mat2.cols)
    }

    var i1, j1
    var mat3 = new Matrix(mat1.rows, mat2.cols)

//    for (i1 = 0; i1 < mat1.rows; i1++ ) {
//        for (j1 = 0; j1 < mat2.cols; j1++ ) {
//            mat3.data[i1][j1] =
//        }
//
//    }




}
