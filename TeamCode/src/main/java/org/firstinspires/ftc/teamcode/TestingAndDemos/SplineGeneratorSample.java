package org.firstinspires.ftc.teamcode.TestingAndDemos;

import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.SplinePath;

public class SplineGeneratorSample {
    public static void main(String [] args) {
        double [][] roots = {
                {0.0, 0.0},
                {0.0, 1.0},
                {1.0, 0.0},
                {1.0, 1.0},
                {2.0, 0.0},
                {2.0, 1.0},
                {3.0, 0.0},
                {3.0, 1.0}

        };

//		double [][] path;
//
//		SplinePath pathGenerator = new SplinePath(roots);
//		path = pathGenerator.generatePath(0.01, 0.0, 1.0);
//
//		System.out.print("Roots:\n[");
//		for(double [] root : roots) {
//			System.out.printf("(%5.3f, %5.3f), ", root[0], root[1]);
//		}
//		System.out.print("]");
//
//		System.out.print("\nPath:\n[");
//		for(double [] point : path) {
//			System.out.printf("(%5.3f, %5.3f), ", point[0], point[1]);
//		}
//		System.out.print("]");

        // Interpolation testing
        double [][] interpolatedRoots =
                new SplinePath(SplinePath.interpolate(roots, 10)).generatePath(0.01, 0, 1);

        System.out.print("Roots:\n[");
        for(double [] root : roots) {
            System.out.printf("(%5.3f, %5.3f), ", root[0], root[1]);
        }
        System.out.print("]\n\n");

        System.out.print("Interpolated path:\n[");
        for(double [] root : interpolatedRoots) {
            System.out.print(String.format("(%5.3f, %5.3f), ", root[0], root[1]));
        }
        System.out.print("]");
    }
}

