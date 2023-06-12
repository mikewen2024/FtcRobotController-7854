package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.MecanumDrive;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

@Config
@TeleOp(name = "Basic OpMode")
public class BasicOpMode extends OpMode {
    // Dashboard Obj
    public static FtcDashboard dashboard;
    TelemetryPacket packet;
    MultipleTelemetry dataDump;

    // Hardware
    public MecanumDrive drive;

    // Auxillary variables (low pass, PID, etc...)
    public double [] previousInputs;
    public static final double LOW_PASS_LATENCY = 0.5;
    private TelemetryPacket telemetryPacket;

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
        drive.NormalDrive(previousInputs [0] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_x,
                - previousInputs [1] * LOW_PASS_LATENCY - (1.0 - LOW_PASS_LATENCY) * gamepad1.left_stick_y,
                previousInputs [2] * LOW_PASS_LATENCY + (1.0 - LOW_PASS_LATENCY) * gamepad1.right_stick_x);

        telemetry();
    }

    public void telemetry(){
        telemetryPacket.fieldOverlay().setFill("pink").fillRect(0, 0, 1000, 500);
        telemetryPacket.put("uwu", 20);
        dashboard.sendTelemetryPacket(packet);

        logGamepad(telemetry, gamepad1, "Raw gamepad inputs");

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
            telemetry.addData("Left stick x", gamepad.left_stick_x);
        }
    }
}
