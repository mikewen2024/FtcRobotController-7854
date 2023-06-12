package org.firstinspires.ftc.teamcode.DriveTrainAndNavigation;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.MecanumDrive;

// Provides different applications of PID

@Config
public class PIDDrive {

    public double [] vars = {0.0, 0.0, 0.0};

    public PIDDrive(){

    }

    public PIDDrive(double p, double i, double d){

    }

    class driveToPoint implements Runnable { // To run to certain location, running as seperate thread should allow other functionalities to be maintained

        // Each 2 long, can expand to 3 later
        double [] targetState;
        double [] currentState;

        double [] delta;
        double [] derivative = {0.0, 0.0, 0.0};
        double [] integral = {0.0, 0.0, 0.0};
        MecanumDrive drive;

        double threshold; // inches within target coordinate, will add radians later

        // Must initiate simple path before running

        public driveToPoint(double [] targetState, double [] currentState, double threshold, MecanumDrive drive){
            for(int i = 0; i < 2; i++){
                this.targetState [i] = targetState [i];
                this.currentState [i] = currentState [i];

                this.delta [i] = this.targetState [i] - this.currentState [i];
            }

            this.threshold = threshold;
            this.drive = drive;
        }

        @Override
        public void run() {
            while(Math.sqrt(this.delta[0]*this.delta[0] + this.delta[1]*this.delta[1]) > threshold){ // Distance to target outside threshold
                // Set powers, normalize



                // Update state
                for(int i = 0; i < 2; i++) {
                    // Insert odometry algorithm call
                }
            }
        }
    }

    public void driveOnPath(){

    }


}
