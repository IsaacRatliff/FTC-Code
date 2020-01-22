package workspace_;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="5 Point Parking (Blue)", group="Linear Opmode")

public class ParkingBlue extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive_0 = null;
    private DcMotor rightDrive_0 = null;
    private DcMotor leftDrive_1 = null;
    private DcMotor rightDrive_1 = null;
    
    private void leftWheels(double spd){
        leftDrive_0.setPower(spd);
        leftDrive_1.setPower(spd);
    }
    
    private void rightWheels(double spd){
        rightDrive_0.setPower(spd);
        rightDrive_1.setPower(spd);
    }

    @Override
    public void runOpMode() {
        double speed = 6.45/4;  // it took 6.45 sec to go 4 tiles at full spd
        double turn90 = 1.8;
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();
        
        leftDrive_0  = hardwareMap.get(DcMotor.class, "leftDrive_0");
        rightDrive_0 = hardwareMap.get(DcMotor.class, "rightDrive_0");
        leftDrive_1  = hardwareMap.get(DcMotor.class, "leftDrive_1");
        rightDrive_1 = hardwareMap.get(DcMotor.class, "rightDrive_1");
        
        leftDrive_0.setDirection(DcMotor.Direction.FORWARD);
        rightDrive_0.setDirection(DcMotor.Direction.REVERSE);
        leftDrive_1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive_1.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // Step through each leg of the path, ensuring that the Auto mode has not been stopped along the way

        // Step 1:  Drive forward 0.5 tiles
        /*leftWheels(1.0);
        rightWheels(1.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5*speed)) {
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 2:  Spin left 90 degrees
        leftWheels(-1.0);
        rightWheels(1.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < turn90)) {
            telemetry.addData("Path", "Leg 2: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }*/

        // Step 3:  Drive Forward 2.0 tiles
        leftWheels(1.0);
        rightWheels(1.0);
        runtime.reset();
        while (opModeIsActive() && (runtime.seconds() < 0.5*speed)) {
            telemetry.addData("Path", "Leg 3: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        // Step 4:  Stop.
        leftWheels(0);
        rightWheels(0);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }
}
