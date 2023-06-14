package org.firstinspires.ftc.teamcode.DriveTrainAndNavigation;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDDrive implements Runnable{ // Needs to start new thread when starting path to enable manipulator functionalities

    // P, I, D for x, y, rotation
    // x and y relative to robot heading, not absolute
    double [] P;
//    double [] I;
//    double [] D;

    // Distance tolerance
    double tolerance; // In, x and y
    double angleTolerance; // Will add later, just using x and y for now

    Telemetry telemetry;
    public MecanumDrive drive;
    public Odometry odometry;
    double [] currentState = {0.0, 0.0, 0.0};
    double [] targetState = {0.0, 0.0, 0.0};
    double [] delta = {0.0, 0.0, 0.0};

    public PIDDrive(double [] P, /*double [] I, double [] D,*/ double tolerance, MecanumDrive drive, Odometry odometry, double [] targetState, Telemetry telemetry){
        for(int i = 0; i < 3; i++){
            this.P [i] = P [i];
//            this.I [i] = I [i];
//            this.D [i] = D [i];

            this.targetState [i] = targetState [i];
        }

        this.tolerance = tolerance;

        // Just copying pointers
        this.drive = drive;
        this.odometry = odometry;
        this.telemetry = telemetry;
    }

    @Override
    public void run() {

    }

    private boolean iteratePID(){ // Just P for now, states represnted by x, y, angle relative to horizontal right
        // Update state and delta
        this.odometry.updatePosition();
        this.odometry.updateTime();

        this.currentState [0] = odometry.getXCoordinate();
        this.currentState [1] = odometry.getYCoordinate();
        this.currentState [2] = odometry.getRotationRadians();
        for(int i = 0; i < 3; i++){
            delta [i] = this.targetState [i] - this.currentState [i];
        }
        if(delta[0]*delta[0] + delta[1]*delta[1] < tolerance){
            return true;
        }

        // If not within tolerance of position
        double maxInputValue = 0.0;
        for(int i = 0; i < 3; i++){
            delta [i] *= P [i]; // Multiply by gain

            if(Math.abs(delta [i]) > maxInputValue){
                maxInputValue = delta [i];
            }
        }
        for(int i = 0; i < 3; i++){
            delta [i] /= maxInputValue;
        }

        this.drive.FieldOrientedDrive(delta [0], delta [1], 0.0, currentState[2], this.telemetry);

        return false;
    }
}
