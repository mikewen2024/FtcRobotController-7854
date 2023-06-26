package org.firstinspires.ftc.teamcode.DriveTrainAndNavigation;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDDrive{
    // 6-14-23 Tuned for Mayhem '22-'23 bot w/ only bottom half,
    private Odometry odometry;
    private Telemetry telemetry;

    // Auxillary variables (low pass, PID, etc...), currently hardcoded since no real need to do otherwise
    public double [] previousInputs;
    public static final double LOW_PASS_LATENCY = 0.5;
    private TelemetryPacket telemetryPacket;

    // PID
    double [] P = {0.15, 0.15, -1.0};
    double [] I = {0.4, 0.4, -0.075};
    final double integralDecay = 0.95;
    double [] D = {0.1, 0.1, -0.05};
    double [] D2 = {0.75, 0.75, -0.05};
    double [] cumulativeError = {0.0, 0.0, 0.0};

    // Navigational variables
    double [] delta = {0.0, 0.0, 0.0};
    double [] targetState;
    double distanceToTarget = 0.0;
    double lastDistanceToTarget = 0.0;

    public  PIDDrive (Odometry odometry, double targetX, double targetY, double targetRadians, Telemetry telemetry){
        this.odometry = odometry;
        this.telemetry = telemetry;

        this.targetState [0] = targetX;
        this.targetState [1] = targetY;
        this.targetState [2] = targetRadians;
    }

    public double [] updatePID(){ // Update as often as possible, preferrably every tick
        // Since "delta" takes into consideration all the PID coefficients and is directly inputted into the drive code, distance to target calculated seperately from delta
        distanceToTarget = (targetState [0] - odometry.getXCoordinate()) * (targetState [0] - odometry.getXCoordinate());
        distanceToTarget += (targetState [1] - odometry.getYCoordinate()) * (targetState [1] - odometry.getYCoordinate());
        distanceToTarget = Math.sqrt(distanceToTarget);
        telemetry.addData("Distance To Target", distanceToTarget);

        // UpdateTime is already inside updatePosition method
        odometry.updatePosition();

        // Output data (was used in testing)
        telemetry.addData("\nPID ======================\nLeft", odometry.leftTicks);
        telemetry.addData("Right", odometry.rightTicks);
        telemetry.addData("Front", odometry.topTicks);

        telemetry.addData("\nx", odometry.getXCoordinate());
        telemetry.addData("y", odometry.getYCoordinate());
        telemetry.addData("angle", odometry.getRotationDegrees());

        telemetry.addData("\nintegral x gain", cumulativeError [0]);
        telemetry.addData("integral y gain", cumulativeError [1]);
        telemetry.addData("angular y gain", cumulativeError [2]);

        // Proportional component, x and y
        delta [0] = (targetState [0] - odometry.getXCoordinate()) * P [0];
        delta [1] = (targetState [1] - odometry.getYCoordinate()) * P [1];
        // Proportional component, angle
        if(Math.abs(targetState [2] - (odometry.getRotationRadians() % (2.0 * Math.PI))) < Math.PI) { // Enables rotation clockwise or counterclockwise, depending on which way is closer to angle target
            delta [2] = (targetState [2] - (odometry.getRotationRadians() % (2.0 * Math.PI))) * P [2];
            // Needs % statement since angle will likely go over 360˚
        }else{
            delta [2] = (targetState [2] + odometry.getRotationRadians() % (2.0 * Math.PI) - (2.0 * Math.PI))  * P [2];
        }

        // Integral component independently calculated, then added to delta, since "delta" has P and D components added already
        cumulativeError [0] += I [0] * delta [0];
        cumulativeError [1] += I [1] * delta [1];
        cumulativeError [2] += I [2] * ((targetState [2] - (odometry.getRotationRadians() % (2.0 * Math.PI))));

        // Will slow robot more by increasing damping term (P term) on approach to target for x and y
        // Meant to solve issue of integral term building up too much when robot starts far away from target, causing robot overshoot
        if(distanceToTarget - lastDistanceToTarget < 0.0){ // If approaching target
            // Profiles d term on approach (jacks up term to increase "braking" gain)
            double XYDterm = (0.9 / (1 + Math.exp(distanceToTarget - 15))) + 0.1;
            delta [0] -= odometry.getVelocity().x * XYDterm;
            delta [1] -= odometry.getVelocity().y * XYDterm;
        }else{ // If overshooting target
            // Overshoot handled here (will affect first tick leaving a target point to go to next target)
            delta [0] -= odometry.getVelocity().x * D2 [0];
            delta [1] -= odometry.getVelocity().y * D2 [1];
        }
        // Turning doesn't have have the same overshooting issues, so just do velocity gain calculations
        delta [2] += odometry.angularVelocity * D [2];

        // Add cumulative error up seperately from PID, since P and D components are reset every tick
        delta [0] += cumulativeError [0];
        delta [1] += cumulativeError [1];
        delta [2] += cumulativeError [2];

        // Since inputting gain values into FieldOrientedDrive (made to take inputs from -1 to 1), need to normalize magnitude of x, y, angle inputs
        double maxInputValue = 0.0;
        delta [0] *= P [0];
        delta [1] *= P [0];
        delta [2] *= P [0];
        for(double num : delta){
            if (Math.abs(num) > maxInputValue) {
                maxInputValue = Math.abs(num);
            }
        }
        if(maxInputValue > 1.05){
            for(int i = 0; i < 3; i++){
                delta [i] *=  0.75 / Math.abs(maxInputValue); // Hardcoded power coefficient to slow down robot during testing
            }
        }

        telemetry.addData("x delta input", delta [0]);
        telemetry.addData("y delta input", delta [1]);
        telemetry.addData("angle delta input", delta [2]);
        telemetry.addLine("==========================");

        // Make sure integral term doesn't go out of control
        cumulativeError [0] *= integralDecay;
        cumulativeError [1] *= integralDecay;
        cumulativeError [2] *= integralDecay;

        // For calculating speed of target approach
        lastDistanceToTarget = distanceToTarget;

        // Return values go into FieldOrientedDrive
        return this.delta;
    }

    public void setTargetState(double targetX, double targetY, double targetRadians){
        this.targetState [0] = targetX;
        this.targetState [1] = targetY;
        this.targetState [2] = targetRadians;
    }
}