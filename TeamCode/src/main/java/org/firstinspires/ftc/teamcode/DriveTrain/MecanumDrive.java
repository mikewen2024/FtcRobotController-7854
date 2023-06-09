package org.firstinspires.ftc.teamcode.DriveTrain;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class MecanumDrive {
    // Motors
    public DcMotorEx FL;
    public DcMotorEx FR;
    public DcMotorEx BL;
    public DcMotorEx BR;

    // Static motor power multiplier constants
    // Assumes all motors pointing outwards
    // Forward is side left clockwise, right side counterclockwise
    // Strafing right is FL clockwise, FR counterclockwise, BL counterclockwise, BR clockwise
    // Turning right is FL clockwise, FR clockwise, BL clockwise, BR clockwise
    public static final double [] FORWARD = {1.0, -1.0, 1.0, -1.0};
    public static final double [] RIGHT = {1.0, -1.0, -1.0, 1.0};
    public static final double [] CLOCKWISE = {1.0, 1.0, 1.0, 1.0};

    // Inputs and power constraints
    private double [] motorInputs;

    public MecanumDrive(HardwareMap hardwareMap){
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        motorInputs = new double [4];
    }

    // Driving code for TeleOp
    public void TeleOpDrive(boolean fieldOriented, double x, double y, double rotation){
        // Field oriented drive
        if(fieldOriented) {

        }else{// Non field oriented drive
            // Linear combination of drive vectors
            for(int i = 0; i < 4; i++){
                motorInputs [i] = ((FORWARD [i] * y) + (RIGHT [i] * x) + (CLOCKWISE [i] * rotation)) / 3.0;
                // Power "normalization" is a placeholder
            }

            // Normalize power inputs
            for(int i = 0; i < 4; i++){
                // Placeholder
            }
        }
    }

    public void telemetry(FtcDashboard dashboard) {

    }
}
