package org.firstinspires.ftc.teamcode.DriveTrainAndNavigation;

import static java.lang.Thread.sleep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

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

    // FR was being weird
    public static final double [] FORWARD = {1.0, 1.0, 1.0, -1.0};
    public static final double [] RIGHT = {1.0, -1.0, -1.0, -1.0};
    public static final double [] CLOCKWISE = {1.0, -1.0, 1.0, 1.0};
    public static final double POWER_MULTIPLIER = 0.75;

    // Inputs and power constraints
    private double [] motorInputs;

    public MecanumDrive(HardwareMap hardwareMap, Telemetry telmeetry){
        FL = hardwareMap.get(DcMotorEx.class, "FL");
        FR = hardwareMap.get(DcMotorEx.class, "FR");
        BL = hardwareMap.get(DcMotorEx.class, "BL");
        BR = hardwareMap.get(DcMotorEx.class, "BR");

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//        FL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0.5, 0.25, 0.25, 0));
//        FR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0.5, 0.25, 0.25, 0));
//        BL.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0.5, 0.25, 0.25, 0));
//        BR.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(0.5, 0.25, 0.25, 0));

        motorInputs = new double [4];
    }

    // Driving code for TeleOp
    public void NormalDrive(double x, double y, double rotation, Telemetry telemetry){
        double maxPowerLevel = 0.0;

        // Linear combination of drive vectors
        for(int i = 0; i < 4; i++){
            motorInputs [i] = ((FORWARD [i] * y) + (RIGHT [i] * x) + (CLOCKWISE [i] * rotation));

            if(Math.abs(motorInputs [i]) > maxPowerLevel){
                maxPowerLevel = motorInputs [i];
            }
        }

        // Normalize power inputs within envelope, dead zone 0.2
        double powerEnvelope = Math.sqrt(motorInputs[0]*motorInputs[0] + motorInputs[1]*motorInputs[1] + motorInputs[2]*motorInputs[2] + motorInputs[3]*motorInputs[3]) / 2.0;
        if(powerEnvelope > 0.2 && maxPowerLevel <= 1.0){
            for(int i = 0; i < 4; i++){
                motorInputs [i] *=  POWER_MULTIPLIER / (2.0 * maxPowerLevel);
            }
        }

        telemetry.addData("Power envelope", powerEnvelope);
        telemetry.addData("Max Power", maxPowerLevel);
        telemetry.addData("X", x);
        telemetry.addData("Y", y);

        telemetry.addData("FL", motorInputs [2]);
        telemetry.addData("FR", motorInputs [1]);
        telemetry.addData("BL", motorInputs [0]);
        telemetry.addData("BR", motorInputs [3]);

        BL.setPower( motorInputs [0]);
        FR.setPower( motorInputs [1]);
        FL.setPower( motorInputs [2]);
        BR.setPower( motorInputs [3]);
    }

    public void FieldOrientedDrive(double x, double y, double rotation, double angle, Telemetry telemetry){ // Angle of front from horizontal right, meant for controller inputs
        double maxPowerLevel = 0.0;

        // Rotate x and y by negative of angle
        double newX = x*Math.cos(-angle) + y*Math.sin(-angle);
        double newY = -x*Math.sin(-angle) + y*Math.cos(angle);

        // Linear combination of drive vectors
        for(int i = 0; i < 4; i++){
            motorInputs [i] = ((FORWARD [i] * newY) + (RIGHT [i] * newX) + (CLOCKWISE [i] * rotation));
            if(Math.abs(motorInputs [i]) > maxPowerLevel){
                maxPowerLevel = motorInputs [i];
            }
        }

        // Normalize power inputs within envelope, dead zone 0.2
        double powerEnvelope = Math.sqrt(motorInputs[0]*motorInputs[0] + motorInputs[1]*motorInputs[1] + motorInputs[2]*motorInputs[2] + motorInputs[3]*motorInputs[3]);
        if(powerEnvelope > 0.2 && maxPowerLevel <= 1.0){
            for(int i = 0; i < 4; i++){
                motorInputs [i] *=  POWER_MULTIPLIER / (2.0 * maxPowerLevel);
            }
        }

        telemetry.addData("Power envelope", powerEnvelope);
        telemetry.addData("Max Power", maxPowerLevel);
        telemetry.addData("X", newX);
        telemetry.addData("Y", newY);

        telemetry.addData("FL", motorInputs [2]);
        telemetry.addData("FR", motorInputs [1]);
        telemetry.addData("BL", motorInputs [0]);
        telemetry.addData("BR", motorInputs [3]);

        BL.setPower(motorInputs [0]);
        FR.setPower(motorInputs [1]);
        FL.setPower(motorInputs [2]);
        BR.setPower(motorInputs [3]);
    }

    public void telemetry(FtcDashboard dashboard) {

    }
}
