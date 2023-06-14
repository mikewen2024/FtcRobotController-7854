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


Expansion hub

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
//    public Odometry odometry;

    // Auxillary variables (low pass, PID, etc...)
    public double [] previousInputs;
    public static final double LOW_PASS_LATENCY = 0.5;
    private TelemetryPacket telemetryPacket;

    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();

        // Drivetrain
        drive = new MecanumDrive(hardwareMap, telemetry);
//        odometry = new Odometry(hardwareMap, 0.0, new Vector2(0.0, 0.0));

        // Aux data
        previousInputs = new double [3]; // left stick x, left stick y, right stick x

        // Dashboard
        packet = new TelemetryPacket();
        dataDump = new MultipleTelemetry();
    }

    @Override
    public void loop() {

        drive.FieldOrientedDrive(1.0, 0.0, 0.0, 0.0, telemetry);

//        odometry.updatePosition();
//        odometry.updateTime();

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
