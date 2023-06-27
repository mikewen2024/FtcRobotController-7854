public class SplinePath {
    public double [] coords = {0.0, 0.0};
    public double [][][] iteratorMatrix; // Use to encapsulate nodes, for n roots, n by n matrix
    public double t = 0.0;

    // Base node components
    /*
    Order:             Nodes:

    0.      root    root    root    root
                \  /    \  /    \  /
    1.          node    node    node
                    \  /    \  /
    2.              node    node
                        \  /
    3.                  node
     */

    public SplinePath(double [][] roots){

        iteratorMatrix = new double [roots.length][roots.length][2];

        for(int i = 0; i < roots.length; i++) { // Copy over roots
            this.iteratorMatrix [0][i] = roots[i].clone();
        }

        // Set nodes for base iteration, t defaults to 0, can set from elsewhere
        for(int i = 1; i < iteratorMatrix.length; i++){ // For each row below the first one
            for(int j = 0; j < iteratorMatrix.length - i; j++) { // Will have 1 less node than previous layer
                this.iteratorMatrix [i][j][0] = this.iteratorMatrix[i - 1][j][0];
                this.iteratorMatrix [i][j][1] = this.iteratorMatrix[i - 1][j][1];
            }
        }
    }

    public double [] getCoords(double time) {

        this.t = time;

        for(int i = 1; i < iteratorMatrix.length; i++) { // Iterate through each layer
            for(int j = 0; j < iteratorMatrix.length - i; j++) { // Update each node of layer
                this.iteratorMatrix [i][j][0] = (1.0 - t) * this.iteratorMatrix[i - 1][j][0] + t * this.iteratorMatrix[i - 1][j + 1][0];
                this.iteratorMatrix [i][j][1] = (1.0 - t) * this.iteratorMatrix[i - 1][j][1] + t * this.iteratorMatrix[i - 1][j + 1][1];
            }
        }

        // Isolates coords from iteratorMatrix
        this.coords[0] = iteratorMatrix[iteratorMatrix.length - 1][0][0];
        this.coords[1] = iteratorMatrix[iteratorMatrix.length - 1][0][1];

        return this.coords;
    }

    public double [][] generatePath(double dt, double startTime, double endTime){ // Assumes user knows bound of 0, 1

        this.t = startTime;
        double [][] path = new double [(int) ((endTime - startTime) / dt) + 1][2];
        int iteration = 0;

        for(double i = startTime; i < endTime; i+= dt) {
            path[iteration][0] = this.getCoords(i)[0];
            path[iteration][1] = this.getCoords(i)[1];

            iteration++;
        }

        return path;
    }

    // Splits up larger path into subpaths b/w each pair of points to increase fitting accuracy, returns full spline path
    // Note: resolution multiplied to # points
    public static double [][] interpolate(double[][] points, int resolution){

        // Iterate each line segment b/w 2 adjacent points, fill in w/ more points
        double [][] interpolatedRoots = new double [(points.length - 1) * resolution][2];

        for(int i = 0; i < points.length - 1; i++) {
            SplinePath auxillaryPath = new SplinePath(new double [][] {points [i], points [i + 1]});

            for(int point = 0; point < resolution; point++) {
                interpolatedRoots [i * resolution + point] = auxillaryPath.generatePath(1.0 / resolution, 0, 1) [point];
            }
        }

        return interpolatedRoots;
    }
}