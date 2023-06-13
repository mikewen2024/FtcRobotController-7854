package org.firstinspires.ftc.teamcode.DriveTrainAndNavigation;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.concurrent.TimeUnit;
@Config
public class Odometry { // TEMPORARY
    double deltaTime = 0;
    double lastTime = 0;

    double inPerTick = 0.0010819; // 1 - 7 recalibrated distances
    double verticalWheelDistance = 4.0; // 1 - 7 recalibrated dimensions
    double lateralWheelDistance = 12; // 1 - 7 recalibrated dimensions

    public Vector2 position = new Vector2();
    Vector2 velocity = new Vector2();

    int lastLeftTicks = 0;
    int deltaLeftTicks = 0;
    double leftDistanceMoved;

    int lastRightTicks = 0;
    int deltaRightTicks = 0;
    double rightDistanceMoved;

    int lastTopTicks = 0;
    int deltaTopTicks = 0;
    double topDistanceMoved;

    public double rotationRadians;

    ElapsedTime elapsedTime;

    public DcMotorEx leftEncoder;
    public DcMotorEx horizontalEncoder;
    public DcMotorEx rightEncoder;

    public Odometry(HardwareMap hardwareMap, double startingAngleRadians, Vector2 startingPosition) {
        elapsedTime = new ElapsedTime();


        leftEncoder = hardwareMap.get(DcMotorEx.class, "FL");
//        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftEncoder.setDirection(DcMotorSimple.Direction.REVERSE);

        rightEncoder = hardwareMap.get(DcMotorEx.class, "FR");
//        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        horizontalEncoder = hardwareMap.get(DcMotorEx.class, "BR");
//        horizontalEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        this.rotationRadians = startingAngleRadians;
        this.position = startingPosition;
    }

    public void updateTime() {
        double currentTime = elapsedTime.time(TimeUnit.MICROSECONDS) / 1000000.0d;
        deltaTime = currentTime - lastTime;
        lastTime = currentTime;
    }

    public int leftTicks;
    public int rightTicks;
    public int topTicks;

    double deltaRadians;

    double forwardMovement;

    double lateralMovementFromRotation;
    double trueLateralMovement;

    double sin;
    double cosine;

    double netX;
    double netY;

    public int ticksPerSecon = 0;

    // Low pass filter

    double prevLeft;
    double prevRight;
    double prevFront;

    public void updatePosition() {
        leftTicks = (int) (0.5 * leftEncoder.getCurrentPosition()+ 0.5 * prevLeft);
        rightTicks = (int) (-0.5 * rightEncoder.getCurrentPosition() + 0.5 * prevRight);
        topTicks = (int) (0.5 * horizontalEncoder.getCurrentPosition() + 0.5 * prevFront);

        deltaLeftTicks = leftTicks - lastLeftTicks;
        deltaRightTicks = rightTicks - lastRightTicks;
        deltaTopTicks = topTicks - lastTopTicks;

        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;
        lastTopTicks = topTicks;

        // raw distance from each encoder
        leftDistanceMoved = inPerTick * deltaLeftTicks;
        rightDistanceMoved = inPerTick * deltaRightTicks;
        topDistanceMoved = inPerTick * deltaTopTicks;

        // 1 - 7 Restored +1/2 rotation trapezoidal integration method
        // angles
        deltaRadians = -2 * getDeltaRotation(leftDistanceMoved, rightDistanceMoved);
        rotationRadians += .5 * deltaRadians;

        forwardMovement = (leftDistanceMoved + rightDistanceMoved) / 2.0;
        trueLateralMovement = topDistanceMoved - deltaRadians * verticalWheelDistance;

        sin = Math.sin(rotationRadians);
        cosine = Math.cos(rotationRadians);

        netX = forwardMovement * cosine - trueLateralMovement * sin;
        netY = forwardMovement * sin + trueLateralMovement * cosine;

        rotationRadians += .5 * deltaRadians;

//        if (false) {
//            netX = forwardMovement * Math.cos(rotationRadians);
//            netY = forwardMovement * Math.sin(rotationRadians);
////                 (D1 + D2) / 2      Orientation Vector
//            // third wheel component
//            netX -= (trueLateralMovement) * (Math.sin(rotationRadians));
//            netY += (trueLateralMovement) * (Math.cos(rotationRadians));
//            //       Horizontal movement       Normal Vector
//        }

        // 1 - 7 Changed signs since was reversed, had to re-swap variables
        this.position.x -= netX;
        this.position.y -= netY;

//        // Temporary
//        AutonomousNew.currentPosition[0] += netX;
//        AutonomousNew.currentPosition[1] += netY;
//
//        MainTeleOp.currentPosition[0] += netX;
//        MainTeleOp.currentPosition[1] += netY;


        velocity.x = netX / deltaTime;
        velocity.y = netY / deltaTime;


    }

    public void setPostion(Vector2 pos) {
        this.position = pos;
    }

    public void setRotation(double rotation) {
        this.rotationRadians = rotation;
    }

    public double getDeltaRotation(double leftChange, double rightChange) {
        return (rightChange - leftChange) / lateralWheelDistance;
    }

    public double getXCoordinate() {
        return position.x;
    }

    public double getYCoordinate() {
        return position.y;
    }

    public double getRotationRadians() {
        return rotationRadians;
    }

    public double getRotationDegrees() {
        return rotationRadians * 180 / Math.PI;
    }

    public Vector2 getVelocity() {
        return velocity;
    }
    public void resetEncoders() {
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        position = new Vector2();
        rotationRadians = 0;

        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontalEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public String positionToString() {return String.format("(%f, %f)", position.x, position.y); }

    public void telemetry(Telemetry telemetry) {
        telemetry.addLine();
        telemetry.addLine("THREE WHEEL ODOMETRY");

        telemetry.addLine(String.valueOf(deltaTime));

        telemetry.addData("Wheel ticks", String.format("%d, %d, %d", leftTicks, rightTicks, topTicks));

        telemetry.addLine("--------");
        telemetry.addLine("POSITION " + position);
        telemetry.addLine("ROTATION " + getRotationDegrees());
        telemetry.addLine("--------");

        telemetry.addLine("Velocity " + velocity.toString());
        telemetry.addData("Left Dead Wheel Position", leftEncoder.getCurrentPosition());
        telemetry.addData("Right Dead Wheel Position", rightEncoder.getCurrentPosition());
        telemetry.addData("Top Dead Wheel Position", horizontalEncoder.getCurrentPosition());
        telemetry.addData("netX", netX);
        telemetry.addData("netY", netY);
        telemetry.addData("trueLateralMovement", trueLateralMovement);
    }

}

