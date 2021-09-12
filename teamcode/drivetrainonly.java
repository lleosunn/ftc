package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="drivetrainonly", group="Linear Opmode")

public class drivetrainonly extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor tl = null;
    private DcMotor tr = null;
    private DcMotor bl = null;
    private DcMotor br = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        tl = hardwareMap.get(DcMotor.class, "tl");
        tr = hardwareMap.get(DcMotor.class, "tr");
        bl = hardwareMap.get(DcMotor.class, "bl");
        br = hardwareMap.get(DcMotor.class, "br");


        tl.setDirection(DcMotor.Direction.FORWARD);
        bl.setDirection(DcMotor.Direction.FORWARD);
        tr.setDirection(DcMotor.Direction.REVERSE);
        br.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            double tlpower    = Range.clip(drive + turn, -1.0, 1.0) ;
            double trpower   = Range.clip(drive - turn, -1.0, 1.0) ;
            double blpower    = Range.clip(drive + turn, -1.0, 1.0) ;
            double brpower    = Range.clip(drive - turn, -1.0, 1.0) ;

            if (gamepad1.left_stick_x < 0) { //strafe left
                tl.setPower(-0.7);
                tr.setPower(0.7);
                bl.setPower(0.7);
                br.setPower(-0.7);
                continue;
            }
            if (gamepad1.left_stick_x > 0) { //strafe right
                tl.setPower(0.7);
                tr.setPower(-0.7);
                bl.setPower(-0.7);
                br.setPower(0.7);
                continue;
            }


            // Send calculated power to wheels
            tl.setPower(tlpower);
            bl.setPower(blpower);
            tr.setPower(trpower);
            br.setPower(brpower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", tlpower, trpower);
            telemetry.update();
        }
    }
}
