package org.firstinspires.ftc.teamcode.TestingAndDemos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.MecanumDrive;
import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.Odometry;
import org.firstinspires.ftc.teamcode.DriveTrainAndNavigation.Vector2;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

/*
 * Test the dashboard gamepad integration.
 * Demo for data output to dashbaord in graphs
 */
@Autonomous
public class DataOutputOpMode extends LinearOpMode {

    static MecanumDrive drive;
    static Odometry odometry;

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
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // START INIT
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Init and start procedure
        odometry = new Odometry(hardwareMap, 0, new Vector2(0, 0));
        drive = new MecanumDrive(hardwareMap, telemetry);

        // Starting odometry thread, but not multithreading rn
//        odometry.run();

        waitForStart();
        // END INIT


        if (isStopRequested()) return;

        while (opModeIsActive()) {
            logGamepad(telemetry, gamepad1, "gamepad1");
            telemetry.update();

            sleep(20);
        }
    }
}