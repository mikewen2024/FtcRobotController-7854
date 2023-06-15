package org.firstinspires.ftc.teamcode.DriveTrainAndNavigation;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class PIDDrive{
    // 6-14-23 Tuned for Mayhem '22-'23 bot w/ only bottom half,

    // Constant vars
    double [] P = {0.3, 0.3, -0.5};
    double [] I = {0.20, 0.20, -0.03};
    final double integralDecay = 0.95;
    double [] D = {0.025, 0.025, -0.015};

    // Navigation vars
    double [] delta = {0.0, 0.0, 0.0};
    double [] targetState = {0.0, 0.0, 0.0};
    double [] cumulativeError = {0.0, 0.0, 0.0};

    // Auxillary object references
    private Odometry odometry;
    private Telemetry telemetry;

    public PIDDrive (double [] targetState, Odometry odometry, Telemetry telemetry){ // ,x y, angle
        for(int i = 0; i < 3; i++){
            this.targetState [i] = targetState [i];
        }
        this.odometry = odometry;
        this.telemetry = telemetry;
    }

    public double [] IteratePID(){ // Yet to tune angle I and D
        this.odometry.updatePosition();
        this.odometry.updateTime();

        this.telemetry.addData("\nPID ======================\nLeft", this.odometry.leftTicks);
        this.telemetry.addData("Right", this.odometry.rightTicks);
        this.telemetry.addData("Front", this.odometry.topTicks);

        this.telemetry.addData("\nx", this.odometry.getXCoordinate());
        this.telemetry.addData("y", this.odometry.getYCoordinate());
        this.telemetry.addData("angle", this.odometry.getRotationDegrees());

        this.telemetry.addData("\nintegral x gain", cumulativeError [0]);
        this.telemetry.addData("integral y gain", cumulativeError [1]);

        // Proportional Gain
        this.delta [0] = (this.targetState [0] - this.odometry.getXCoordinate()) * P [0];
        this.delta [1] = (this.targetState [1] - this.odometry.getYCoordinate()) * P [1];
        // Angle Proportional Gain
        if(Math.abs(this.targetState [2] - (this.odometry.getRotationRadians() % (2.0 * Math.PI))) < Math.PI) {
            this.delta [2] = (this.targetState [2] - (this.odometry.getRotationRadians() % (2.0 * Math.PI))) * P [2];
        }else{
            this.delta [2] = (this.targetState [2] + this.odometry.getRotationRadians() % (2.0 * Math.PI) - (2.0 * Math.PI))  * P [2];
        }

        // Integral Gain
        this.cumulativeError [0] += I [0] * delta [0];
        this.cumulativeError [1] += I [0] * delta [1];
        this.delta [0] += this.cumulativeError [0];
        this.delta [1] += this.cumulativeError [1];

        // Derivative Gain
        this.delta [0] -= this.odometry.getVelocity().x * D [0];
        this.delta [1] -= this.odometry.getVelocity().y * D [1];

        // Made mistake in initial testing w/ multiplying P, since already tuned will replicate steps here
        this.delta [0] *= P [0];
        this.delta [1] *= P[1];
        this.delta [2] *= Math.abs(P [2]);

        // Normalize inputs to 1.0
        double maxInputValue = 0.0;
        for(double num : delta){
            if (Math.abs(num) > maxInputValue) {
                maxInputValue = Math.abs(num);
            }
        }
        if(maxInputValue > 1.0){
            for(int i = 0; i < 3; i++){
                this.delta [i] /= maxInputValue;
            }
        }

        this.telemetry.addData("x delta input", this.delta [0]);
        this.telemetry.addData("y delta input", this.delta [1]);
        this.telemetry.addData("angle delta input\n==========================\n", this.delta [2]);

        this.cumulativeError [0] *= integralDecay;
        this.cumulativeError [1] *= integralDecay;

        return this.delta;
    }
}