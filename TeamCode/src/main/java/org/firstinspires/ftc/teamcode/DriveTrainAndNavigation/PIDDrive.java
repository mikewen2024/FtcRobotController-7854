package org.firstinspires.ftc.teamcode.DriveTrainAndNavigation;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.MecanumDrive;

// Provides different applications of PID

@Config
public class PIDDrive {

    public double [] vars = {0.0, 0.0, 0.0};
    public final double integralDecayConst = 0.9;
    ElapsedTime timer;

    public PIDDrive(double p, double i, double d){
        this.vars [0] = p;
        this.vars [1] = i;
        this.vars [2] = d;
    }

    class driveToPoint implements Runnable { // To run to certain location, running as seperate thread should allow other functionalities to be maintained

        // Each 2 long, can expand to 3 later
        double [] targetState;
        double [] currentState;
        double [] previousState;

        double [] delta;
        double [] derivative = {0.0, 0.0, 0.0};
        double [] integral = {0.0, 0.0, 0.0};
        MecanumDrive drive;
        Odometry odometry;
        double threshold; // inches within target coordinate, will add radians later

        // Must initiate simple path before running
        public driveToPoint(double [] targetState, double [] currentState, double threshold, MecanumDrive drive, Odometry odometry){
            for(int i = 0; i < 2; i++){
                this.targetState [i] = targetState [i];
                this.currentState [i] = currentState [i];

                this.delta [i] = this.targetState [i] - this.currentState [i];
            }

            this.threshold = threshold;
            this.drive = drive;
            this.odometry = odometry;
        }

        @Override
        public void run() {
            while(Math.sqrt(this.delta[0]*this.delta[0] + this.delta[1]*this.delta[1]) > threshold){ // Distance to target outside threshold
                // Set powers, normalize to [-1.0, 1.0]


                for(int i = 0; i < 2; i++){
                    this.previousState [i] = this.currentState [i];
                }
                // Update state
                this.odometry.updatePosition();
                this.currentState [0] = odometry.getXCoordinate();
                this.currentState [1] = odometry.getYCoordinate();
                // Add radians later
                for(int i = 0; i < 2; i++) {
                    this.delta [i] = this.targetState [i] - this.currentState [i];

                    this.derivative [i] = (this.currentState [i] - this.previousState [i]) / timer.seconds(); // in / sec
                    this.integral [i] *= integralDecayConst;
                    this.integral [i] += this.delta [i] * timer.seconds();

                }
                timer.reset();
            }
        }
    }

    public void driveOnPath(){

    }


}
