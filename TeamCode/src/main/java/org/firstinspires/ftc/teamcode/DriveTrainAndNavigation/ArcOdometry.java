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
public class ArcOdometry implements Runnable{ // Odometry class but with arcs, modifies relative x and y calculations

    //All variables mentioned here will be addessed as "this.VARIABLE_NAME"

    //Constants
    double inPerTick = 0.0010819; // 1 - 7 recalibrated distances\\
    double verticalWheelDistance = 4.0; // 1 - 7 recalibrated dimensions
    double lateralWheelDistance = 12; // 1 - 7 recalibrated dimensions


    //Tracking Time
    double deltaTime = 0;
    double lastTime = 0;
    ElapsedTime elapsedTime;

    //Pose Tracking Variables
    public Vector2 position = new Vector2();
    Vector2 velocity = new Vector2();
    public double angularVelocity = 0.0;
    public double rotationRadians;

    //Odometry Wheel Variables
    int lastLeftTicks = 0;
    int deltaLeftTicks = 0;
    double leftDistanceMoved;

    int lastRightTicks = 0;
    int deltaRightTicks = 0;
    double rightDistanceMoved;

    int lastTopTicks = 0;
    int deltaTopTicks = 0;
    double topDistanceMoved;


    //Hardware Variables
    public DcMotorEx leftEncoder;
    public DcMotorEx horizontalEncoder;
    public DcMotorEx rightEncoder;


    //Intermediate Variables OwO

    //Tick readings for encoders
    public int leftTicks;
    public int rightTicks;
    public int topTicks;

    //Angle change
    double deltaRadians;

    //Avg Distance of left and right encoders
    double forwardMovement;

    //Middle encoder movement from rotation
    double lateralMovementFromRotation;
    double trueLateralMovement;

    //sin and cos of robot angle from horizontal right (unit circle style)
    double rotSin;
    double rotCosine;

    //Change X and Y position of robot
    double netX;
    double netY;


    // Low pass filter used to smooth position
    double prevLeft = 0.0;
    double prevRight = 0.0;
    double prevFront = 0.0;

    // For arc-based odometry, if angle change is too small, will need to revert to linear approximation due to overflow errors
    public static final double ARC_ODOMETRY_ANGLE_THRESHOLD = 0.01; // Radians

    public ArcOdometry(HardwareMap hardwareMap, double startingAngleRadians, Vector2 startingPosition) {
        OpModeIsActive = true;
        elapsedTime = new ElapsedTime();

        //Initialize Motors
        leftEncoder = hardwareMap.get(DcMotorEx.class, "FL");
        rightEncoder = hardwareMap.get(DcMotorEx.class, "FR");
        horizontalEncoder = hardwareMap.get(DcMotorEx.class, "BR");

        //Reset Position
        this.rotationRadians = startingAngleRadians;
        this.position = startingPosition;
    }

    public void updatePosition() {

        //Updating Time
        double currentTime = elapsedTime.time(TimeUnit.MICROSECONDS) / 1000000.0d;
        deltaTime = currentTime - lastTime;
        lastTime = currentTime;

        //Implementing low pass filter for smoothing
        //Note: change coefficients to make filter more/less responsive
        leftTicks = (int) (0.5 * leftEncoder.getCurrentPosition()+ 0.5 * prevLeft);
        rightTicks = (int) (-0.5 * rightEncoder.getCurrentPosition() + 0.5 * prevRight);
        topTicks = (int) (0.5 * horizontalEncoder.getCurrentPosition() + 0.5 * prevFront);

        //calculate change in tick reading
        deltaLeftTicks = leftTicks - lastLeftTicks;
        deltaRightTicks = rightTicks - lastRightTicks;
        deltaTopTicks = topTicks - lastTopTicks;

        //update last enocder values with new low pass filter values
        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;
        lastTopTicks = topTicks;

        // raw distance from each encoder
        leftDistanceMoved = inPerTick * deltaLeftTicks;
        rightDistanceMoved = inPerTick * deltaRightTicks;
        topDistanceMoved = inPerTick * deltaTopTicks;

        // calculate change in angles
        deltaRadians = -2 * getDeltaRotation(leftDistanceMoved, rightDistanceMoved);
        angularVelocity = deltaRadians / deltaTime;
        rotationRadians += .5 * deltaRadians; // Trapezoidal approximation

        // Arc-based odometry differs in forward and lateral movement calculation
        // Assumes circular arc movement, localize to robot x and y, then input as forward and lateral movement
        if(deltaRadians < ARC_ODOMETRY_ANGLE_THRESHOLD) { // Case for resorting to linear approximations w/ too small angle changes
            forwardMovement = (leftDistanceMoved + rightDistanceMoved) / 2.0;
            //Actual horizontal movement without rotational influence
            trueLateralMovement = topDistanceMoved - deltaRadians * verticalWheelDistance;

            //assigning sin and cos of rotation
            rotSin = Math.sin(rotationRadians);
            rotCosine = Math.cos(rotationRadians);

            //Calculating change X and Y position of robot
            netX = forwardMovement * rotCosine - trueLateralMovement * rotSin;
            netY = forwardMovement * rotSin + trueLateralMovement * rotCosine;

            rotationRadians += .5 * deltaRadians; // Trapezoidal approximation
        }else{ // Arc approximation
            // Note: turn radius for forward movement is (L + R) / (2 * ∆radians)
            // Radius for front wheel component here (2nd component) is kinda sketchy, placeholder turn radius
            double turnRadius = (leftDistanceMoved + rightDistanceMoved) / (2.0 * deltaRadians);

            forwardMovement = Math.sin(rotationRadians) * turnRadius + // Left and right wheels component
                    (((turnRadius + verticalWheelDistance) * Math.cos(deltaRadians)) - (turnRadius + verticalWheelDistance)); // Front wheel component

            trueLateralMovement = (turnRadius - (turnRadius * Math.cos(deltaRadians))) - // Left and right wheels component
                    ((turnRadius + verticalWheelDistance) * Math.sin(deltaRadians)); // Front wheel component

            //assigning sin and cos of rotation
            rotSin = Math.sin(rotationRadians);
            rotCosine = Math.cos(rotationRadians);

            //Calculating change X and Y position of robot
            netX = forwardMovement * rotCosine - trueLateralMovement * rotSin;
            netY = forwardMovement * rotSin + trueLateralMovement * rotCosine;
        }
        //Calculating final x and y
        //Note: Changed signs since was reversed, had to re-swap variables
        this.position.x += netX;
        this.position.y += netY;

        //getting x and y velocities
        velocity.x = netX / deltaTime;
        velocity.y = netY / deltaTime;
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


    public void setPostion(Vector2 pos) {
        this.position = pos;
    }

    public void setRotation(double rotation) {
        this.rotationRadians = rotation;
    }

    public double getDeltaRotation(double leftChange, double rightChange) {
        return -(rightChange - leftChange) / lateralWheelDistance;
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

// Will be used later for multithreading

    // For multithreading
    public boolean OpModeIsActive;
    @Override
    public void run() { // Run, ideally at a regulated rate, the odometry algorithm
        // Will assume odometry object is already initialized
        // OpModeIsActive variable for emergency stop
        while(OpModeIsActive){
            this.updatePosition();
        }
    }
}