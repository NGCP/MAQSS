
function max(array) {
    /* Function to find the value and index of the maximum element in an 1D array of numbers

        Input:
            array - 1 dimensional array
        Output:
            returns an array with [maxValue, index of maxValue in array]
    */

    if (array.length === 0) throw "Empty array passed into max function";

    var val = [array[0], 0];
    var i1;

    for (i1 = 1; i1 < array.length; i1++) {
        if (array[i1] > val[0]) {
            val[1] = i1;
            val[0] = array[i1];
        }
    }
    return val;
}

function min(array) {
    /* Function to find the value and index of the maximum element in an 1D array of numbers
        Input:
            array - 1 dimensional array
        Output:
            returns an array with [minValue, index of minValue in array]
    */
    if (array.length === 0) throw "Empty array passed into min function";

    var val = [array[0], 0];
    var i1;
    for (i1 = 1; i1 < array.length; i1++) {
        if (array[i1] < val[0]) {
            val[0] = array[i1];
            val[1] = i1;
        }
    }
    return val
}

var swap = function (array, pos1, pos2) {
    // Function to swap two positions in a 1D array. Takes the array by reference not by value
    var temp = array[pos1];
    array[pos1] = array[pos2];
    array[pos2] = temp;
};

function range(start, stop, step) {
    if (typeof stop == 'undefined') {
        // one param defined
        stop = start;
        start = 0;
    }

    if (typeof step == 'undefined') {
        step = 1;
    }

    if ((step > 0 && start >= stop) || (step < 0 && start <= stop)) {
        return [];
    }

    var result = [];
    for (var i = start; step > 0 ? i < stop : i > stop; i += step) {
        result.push(i);
    }

    return result;
};

function onlyUnique(value, index, self) {
    /* Helper function to find unique values in array

      Usage:
      var arrayName = [1, 2, 3]
      var uniqueElements = arrayName.filter( onlyUnique )

      // returns uniqueElements = [1, 2, 3]
    */
    return self.indexOf(value) === index;
}

var heapsPermute = function (array, output, n) {
    /* Function to create all permutations of array and store the permutations as elements of output
        NOTE: This function is O(n!) and should not be run for array lengths approaching or greater than 10

        Input:
            array   - 1 dimensional array of numbers or strings
            output  - 1 dimensional array used to store all permutations
            n       - internal recursion variable. Do not pass if calling function
    */

    // TODO: Implement algorithm more efficient than O(n!)
    var i1, j1;
    n = n || array.length; // set n default to array.length

    if (n > 9) throw "Permutations requested for array with length > 9 (Permutation is O(n!))"

    if (n === 1) {
        // push to output array as a hard copy
        output.push(array.slice(0))
    } else {
        for (i1 = 1; i1 <= n; i1++) {
            heapsPermute(array, output, n - 1);
            if (n % 2) {
                j1 = 1;
            } else {
                j1 = i1;
            }
            swap(array, j1 - 1, n - 1); // -1 to account for javascript zero-indexing
        }
    }
};
