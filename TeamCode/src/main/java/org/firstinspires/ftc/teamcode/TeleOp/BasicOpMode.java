package org.firstinspires.ftc.teamcode.TeleOp;

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

    // Hardware
    public MecanumDrive drive;
    public Odometry odometry;

    // Auxillary variables (low pass, PID, etc...)
    public double [] previousInputs;
    public static final double LOW_PASS_LATENCY = 0.5;
    private TelemetryPacket telemetryPacket;

    double [] P = {0.3, 0.3, -0.5};
    double [] I = {0.20, 0.20, -0.03};
    double [] cumulativeError = {0.0, 0.0, 0.0};
    final double integralDecay = 0.95;
    double [] D = {0.025, 0.025, -0.015};
    double [] delta = {0.0, 0.0, 0.0};
    double [] targetState = {5.0, 5.0, 0.0};

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();

        // Drivetrain
        drive = new MecanumDrive(hardwareMap, telemetry);
        odometry = new Odometry(hardwareMap, Math.PI / 2.0, new Vector2(0.0, 0.0));

        // Aux data
        previousInputs = new double [3]; // left stick x, left stick y, right stick x

        // Dashboard
        packet = new TelemetryPacket();
        dataDump = new MultipleTelemetry();
    }

    @Override
    public void loop() {
        if(gamepad1.x){
            odometry.resetEncoders();
        }

        odometry.updatePosition();
        odometry.updateTime();

        telemetry.addData("\nPID ======================\nLeft", odometry.leftTicks);
        telemetry.addData("Right", odometry.rightTicks);
        telemetry.addData("Front", odometry.topTicks);

        telemetry.addData("\nx", odometry.getXCoordinate());
        telemetry.addData("y", odometry.getYCoordinate());
        telemetry.addData("angle", odometry.getRotationDegrees());

        telemetry.addData("\nintegral x gain", cumulativeError [0]);
        telemetry.addData("integral y gain", cumulativeError [1]);

        delta [0] = (targetState [0] - odometry.getXCoordinate()) * P [0];
        delta [1] = (targetState [1] - odometry.getYCoordinate()) * P [1];

        cumulativeError [0] += I [0] * delta [0];
        cumulativeError [1] += I [0] * delta [1];

        if(Math.abs(targetState [2] - (odometry.getRotationRadians() % (2.0 * Math.PI))) < Math.PI) {
            delta [2] = (targetState [2] - (odometry.getRotationRadians() % (2.0 * Math.PI))) * P [2];
        }else{
            delta [2] = (targetState [2] + odometry.getRotationRadians() % (2.0 * Math.PI) - (2.0 * Math.PI))  * P [2];
        }

        delta [0] -= odometry.getVelocity().x * D [0];
        delta [1] -= odometry.getVelocity().y * D [1];

        delta [0] += cumulativeError [0];
        delta [1] += cumulativeError [1];
//        delta[2] -= D [2] * odometry.angularVelocity;


        double maxInputValue = 0.0;
        for(double num : delta){
            if (Math.abs(num) > maxInputValue) {
                maxInputValue = Math.abs(num);
            }
        }
        if(maxInputValue > 1.0){
            for(int i = 0; i < 3; i++){
                delta [i] /= maxInputValue;
            }
        }

        telemetry.addData("x delta input", delta [0]);
        telemetry.addData("y delta input", delta [1]);
        telemetry.addData("angle delta input\n==========================\n", delta [2]);

//        drive.FieldOrientedDrive(delta [0] * P [0],
//                delta [1] * P [1],
//                delta [2] * P [2],
//                odometry.getRotationRadians(),
//                telemetry);

        cumulativeError [0] *= integralDecay;
        cumulativeError [1] *= integralDecay;

//        drive.NormalDrive(1.0, 0.0, 0.0, telemetry);

        // Handle controller inputs with low pass filter and input into drive
//        if(gamepad1.left_bumper){
            drive.FieldOrientedDrive(previousInputs [0] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_x,
                    - previousInputs [1] * LOW_PASS_LATENCY - (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_y,
                    previousInputs [2] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.right_stick_x,
                    odometry.getRotationRadians(), telemetry);
            telemetry.addLine("Field Oriented");
//        }else{
//            drive.NormalDrive(previousInputs [0] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_x,
//                    - previousInputs [1] * LOW_PASS_LATENCY - (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_y,
//                    previousInputs [2] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.right_stick_x,
//                    telemetry);
//        }

//        previousInputs [0] = gamepad1.left_stick_x;
//        previousInputs [1] = gamepad1.left_stick_y;
//        previousInputs [2] = gamepad1.right_stick_x;

//        telemetry();
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
