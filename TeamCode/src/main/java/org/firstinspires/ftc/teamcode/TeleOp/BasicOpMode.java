package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.canvas.Spline;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.MecanumDrive;
import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.Odometry;
import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.SplinePath;
import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.Vector2;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

/*
Control hub:
Motor Ports:
0. BL, left encoder
1. BR, front encoder
2. FL
3. FR, right encoder
 */

@Config
@TeleOp(name = "Basic OpMode")
public class BasicOpMode extends OpMode {
    // Dashboard Obj
    public static FtcDashboard dashboard;
    TelemetryPacket packet;
    MultipleTelemetry dataDump;
    ElapsedTime timer;

    // Hardware
    public MecanumDrive drive;
    public Odometry odometry;

    // Auxillary variables (low pass, PID, etc...)
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

    double [] delta = {0.0, 0.0, 0.0};
    double [] targetState = {0.0, 40.0, Math.PI / 2.0};

    // Spline driving
    public SplinePath path;
    public double [][] roots = {
            {0.0, 0.0},
            {0.0, 40.0}
    };
    public double [][] interpolatedRoots;
    public int rootIndex = 0;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();
        timer = new ElapsedTime();
        timer.startTime();
        interpolatedRoots = new SplinePath(SplinePath.interpolate(roots, 50)).generatePath(0.001, 0, 1);

        // Drivetrain
        drive = new MecanumDrive(hardwareMap, telemetry);
        odometry = new Odometry(hardwareMap, Math.PI / 2.0, new Vector2(0.0, 0.0));

        // Aux data
        previousInputs = new double [3]; // left stick x, left stick y, right stick x

        // Dashboard
        packet = new TelemetryPacket();
        dataDump = new MultipleTelemetry();
    }
    double distanceToTarget = 0.0;
    double lastDistanceToTarget = 0.0;

    @Override
    public void loop() {
        if(gamepad1.x){
            odometry.resetEncoders();
        }
        // Spline driving
        rootIndex -= (int) (10 * gamepad1.left_stick_y);
        telemetry.addData("Root Index", rootIndex);
        telemetry.addData("Path resolution", interpolatedRoots.length);

        if(rootIndex >= 0 && rootIndex < interpolatedRoots.length){
            targetState [0] = interpolatedRoots [rootIndex][0];
            targetState [1] = interpolatedRoots [rootIndex][0];
            telemetry.addLine(String.format("Target coords : (%4.2d, %4.2d)", interpolatedRoots [rootIndex][0], interpolatedRoots [rootIndex][1]));

            targetState [2] = (1.0 - ((double) (rootIndex) / interpolatedRoots.length)) * Math.PI / 2.0;
        }

        // Routine ig
        distanceToTarget = (targetState [0] - odometry.getXCoordinate()) * (targetState [0] - odometry.getXCoordinate());
        distanceToTarget += (targetState [1] - odometry.getYCoordinate()) * (targetState [1] - odometry.getYCoordinate());
        distanceToTarget = Math.sqrt(distanceToTarget);

        telemetry.addData("Distance To Target", distanceToTarget);
//        telemetry.addData("Target States Reached", targetStatesReached);
//        if(distanceToTarget
//                < 1.2){ // 1.2 In. margin of error
//            targetStatesReached++;
//        }
//        switch(targetStatesReached) {
//            case 2:
//                targetState [0] = 40.0;
//                targetState [2] = -Math.PI / 2.0;
//                break;
//            case 3:
//                targetState [1] = 0.0;
//                targetState [2] = -Math.PI;
//                break;
//            case 4:
//                targetState [0] = 0.0;
//                targetState [2] = Math.PI / 2.0;
//                break;
//            default:
//                break;
//        }

        odometry.updatePosition();

        telemetry.addData("\nPID ======================\nLeft", odometry.leftTicks);
        telemetry.addData("Right", odometry.rightTicks);
        telemetry.addData("Front", odometry.topTicks);

        telemetry.addData("\nx", odometry.getXCoordinate());
        telemetry.addData("y", odometry.getYCoordinate());
        telemetry.addData("angle", odometry.getRotationDegrees());

        telemetry.addData("\nintegral x gain", cumulativeError [0]);
        telemetry.addData("integral y gain", cumulativeError [1]);
        telemetry.addData("angular y gain", cumulativeError [2]);

        delta [0] = (targetState [0] - odometry.getXCoordinate()) * P [0];
        delta [1] = (targetState [1] - odometry.getYCoordinate()) * P [1];

        cumulativeError [0] += I [0] * delta [0];
        cumulativeError [1] += I [1] * delta [1];
        cumulativeError [2] += I [2] * ((targetState [2] - (odometry.getRotationRadians() % (2.0 * Math.PI))));

        if(Math.abs(targetState [2] - (odometry.getRotationRadians() % (2.0 * Math.PI))) < Math.PI) {
            delta [2] = (targetState [2] - (odometry.getRotationRadians() % (2.0 * Math.PI))) * P [2];
        }else{
            delta [2] = (targetState [2] + odometry.getRotationRadians() % (2.0 * Math.PI) - (2.0 * Math.PI))  * P [2];
        }

        if(distanceToTarget - lastDistanceToTarget < 0.0){
            // Power profiling on approach
            double XYDterm = (0.9 / (1 + Math.exp(distanceToTarget - 15))) + 0.1;
            delta [0] -= odometry.getVelocity().x * XYDterm;
            delta [1] -= odometry.getVelocity().y * XYDterm;
        }else{
            // Overshoot handled here (will affect first tick leaving a target point to go to next target)
            delta [0] -= odometry.getVelocity().x * D2 [0];
            delta [1] -= odometry.getVelocity().y * D2 [1];
        }

        delta [2] += odometry.angularVelocity * D [2];

        delta [0] += cumulativeError [0];
        delta [1] += cumulativeError [1];
        delta [2] += cumulativeError [2];

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
                delta [i] /= Math.abs(maxInputValue);
            }
        }

        telemetry.addData("x delta input", delta [0]);
        telemetry.addData("y delta input", delta [1]);
        telemetry.addData("angle delta input", delta [2]);
        telemetry.addLine("==========================");

        drive.FieldOrientedDrive(delta [0] * 0.75, delta [1] * 0.75, delta [2] * 0.75,
                odometry.getRotationRadians(),
                telemetry);

        cumulativeError [0] *= integralDecay;
        cumulativeError [1] *= integralDecay;
        cumulativeError [2] *= integralDecay;

        lastDistanceToTarget = distanceToTarget;

//        drive.NormalDrive(1.0, 0.0, 0.0, telemetry);

        // Handle controller inputs with low pass filter and input into drive

//        if(gamepad1.left_bumper){
//            drive.FieldOrientedDrive(previousInputs [0] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_x,
//                    - previousInputs [1] * LOW_PASS_LATENCY - (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_y,
//                    previousInputs [2] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.right_stick_x,
//                    odometry.getRotationRadians(), telemetry);
//            telemetry.addLine("Field Oriented");
//        }else{
//            drive.NormalDrive(previousInputs [0] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_x,
//                    - previousInputs [1] * LOW_PASS_LATENCY - (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_y,
//                    previousInputs [2] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.right_stick_x,
//                    telemetry);
//        }
//
//        previousInputs [0] = gamepad1.left_stick_x;
//        previousInputs [1] = gamepad1.left_stick_y;
//        previousInputs [2] = gamepad1.right_stick_x;

//        telemetry();

        // Line following
    }

    public void telemetry(){
//        telemetryPacket.fieldOverlay().setFill("pink").fillRect(0, 0, 1000, 500);
//        telemetryPacket.put("uwu", 20);
//        dashboard.sendTelemetryPacket(packet);

//        logGamepad(telemetry, gamepad1, "Raw gamepad inputs");

        telemetry.update();
    }

    private static void logGamepad(Telemetry telemetry, Gamepad gamepad, String prefix) {
        telemetry.addData(prefix + "Synthetic",
                gamepad.getGamepadId() == Gamepad.ID_UNASSOCIATED);
        for (Field field : gamepad.getClass().getFields()) {
            if (Modifier.isStatic(field.getModifiers())) continue;

            try {
                telemetry.addData(prefix + field.getName(), field.get(gamepad));
            } catch (IllegalAccessException e) {
                // ignore for now
            }

            // Adding data:
//            telemetry.addData("Left stick x", gamepad.left_stick_x);
        }
    }
}
