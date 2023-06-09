package org.firstinspires.ftc.teamcode.TeleOp;

import android.hardware.HardwareBuffer;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveTrain.MecanumDrive;

@Config
@TeleOp(name = "")
public class BasicOpMode extends OpMode {
    // Dashboard Obj
    public static FtcDashboard dashboard;
    TelemetryPacket packet;
    MultipleTelemetry dataDump;

    // Hardware
    public HardwareMap hardwareMap;
    public MecanumDrive drive;

    // Auxillary variables (low pass, PID, etc...)
    public double [] previousInputs;
    public static final double LOW_PASS_LATENCY = 0.5;


    @Override
    public void init() {
        dashboard = FtcDashboard.getInstance();

        // Drivetrain
        drive = new MecanumDrive(hardwareMap);

        // Aux data
        previousInputs = new double [3]; // left stick x, left stick y, right stick x

        // Dashboard
        dashboard = FtcDashboard.getInstance();
        packet = new TelemetryPacket();
        dataDump = new MultipleTelemetry();
    }

    @Override
    public void loop() {
        // Handle controller inputs with low pass filter and input into drive
        drive.TeleOpDrive(false,
                previousInputs [0] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_x,
                - previousInputs [1] * LOW_PASS_LATENCY - (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_y,
                previousInputs [2] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.right_stick_x);

        //
    }

    public void telemetry(){
        packet.fieldOverlay().setFill("pink").fillRect(0, 0, 50, 50);
        packet.put("uwu", 20);

        dashboard.sendTelemetryPacket(packet);
        telemetry.update();
    }
}
