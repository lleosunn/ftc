package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="practicecode", group="Linear Opmode")

public class practicecode extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor fl = null;
    private DcMotor fr = null;
    private DcMotor lb = null;
    private DcMotor rb = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        fl = hardwareMap.get(DcMotor.class, "fl");
        fr = hardwareMap.get(DcMotor.class, "fr");
        lb = hardwareMap.get(DcMotor.class, "lb");
        rb = hardwareMap.get(DcMotor.class, "rb");

        fl.setDirection(DcMotor.Direction.FORWARD);
        fr.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.right_stick_x;
            double flpower    = Range.clip(drive + turn, -1.0, 1.0) ;
            double frpower   = Range.clip(drive - turn, -1.0, 1.0) ;
            double lbpower    = Range.clip(drive + turn, -1.0, 1.0) ;
            double rbpower    = Range.clip(drive - turn, -1.0, 1.0) ;

            if (gamepad1.left_stick_x < 0) { //strafe left
                fl.setPower(-0.7);
                fr.setPower(0.7);
                lb.setPower(0.7);
                rb.setPower(-0.7);
                continue;
            }
            if (gamepad1.left_stick_x > 0) { //strafe right
                fl.setPower(0.7);
                fr.setPower(-0.7);
                lb.setPower(-0.7);
                rb.setPower(0.7);
                continue;
            }
            // Send calculated power to wheels
            fl.setPower(flpower);
            fr.setPower(frpower);
            lb.setPower(lbpower);
            rb.setPower(rbpower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", flpower, frpower);
            telemetry.update();
        }
    }
}


