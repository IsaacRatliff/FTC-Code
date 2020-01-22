package workspace_;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import java.util.Arrays;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name="AutoBeans", group="Linear Opmode")
public class AutoBeans extends LinearOpMode {
   
    private DcMotor leftDrive_0 = null;
    private DcMotor rightDrive_0 = null;
    private DcMotor leftDrive_1 = null;
    private DcMotor rightDrive_1 = null;
    private DcMotor arm = null;
    private Servo claw1 = null;
    private Servo claw2 = null;
    
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Beans has been initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive_0  = hardwareMap.get(DcMotor.class, "leftDrive_0");
        rightDrive_0 = hardwareMap.get(DcMotor.class, "rightDrive_0");
        leftDrive_1  = hardwareMap.get(DcMotor.class, "leftDrive_1");
        rightDrive_1 = hardwareMap.get(DcMotor.class, "rightDrive_1");
        arm = hardwareMap.get(DcMotor.class, "arm");
        claw1 = hardwareMap.get(Servo.class, "claw1");
        claw2 = hardwareMap.get(Servo.class, "claw2");
        //Setting directions for motors and servo position to start
        leftDrive_0.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftDrive_0.setDirection(DcMotor.Direction.FORWARD);
        rightDrive_0.setDirection(DcMotor.Direction.REVERSE);
        leftDrive_1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive_1.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.FORWARD);
        claw1.setPosition(0.0);
        claw2.setPosition(0.0);
        
        int posX = 1;
        int posY = 1;
        
        telemetry.addData("Beans:", "waiting");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Beans:", "running");
        telemetry.update();
        
        //11 ticks to go 1 cm
        
        //Step 1: Move forward .5 meter to block
        while(posX != 65 && posY != 50){
            while(posX != 65){
                if(true)
                {
                //MoveX
                    rightDrive_0.setPower(.5);
                    rightDrive_1.setPower(.5);
                    leftDrive_0.setPower(.5);
                    leftDrive_1.setPower(.5);
                    posX += ((rightDrive_0.getCurrentPosition()+
                    rightDrive_1.getCurrentPosition()+leftDrive_0.getCurrentPosition()+
                    leftDrive_1.getCurrentPosition()/4)/11);
                 }
                telemetry.addData("Coords:", posX+","+posY);
                telemetry.update();
            }
            while(posY != 50){
                //MoveY
                if(true)
                {
                    rightDrive_0.setPower(.5);
                    rightDrive_1.setPower(-.5);
                    leftDrive_0.setPower(-.5);
                    leftDrive_1.setPower(.5);
                    posY += ((rightDrive_0.getCurrentPosition()+
                    -rightDrive_1.getCurrentPosition()+-leftDrive_0.getCurrentPosition()+
                    leftDrive_1.getCurrentPosition()/4)/11);
                }
                telemetry.addData("Coords:", posX+","+posY);
                telemetry.update();
            }
            rightDrive_0.setPower(0);
            rightDrive_1.setPower(0);
            leftDrive_0.setPower(0);
            leftDrive_1.setPower(0);
        }
        //Step 2 Grab a block.
        arm.setPower(.25);
        sleep(50);
        arm.setPower(0);
        claw1.setPosition(0.5);
        claw2.setPosition(0.5);
        
        //Step 3 Reverse and strafe right to other zone
        /*
        while(posX != 65 && posY != 50){
            while(posX != 65){
                if(true)
                {
                //MoveX
                    rightDrive_0.setPower(.5);
                    rightDrive_1.setPower(.5);
                    leftDrive_0.setPower(.5);
                    leftDrive_1.setPower(.5);
                    posX = ((rightDrive_0.getCurrentPosition()+
                    rightDrive_1.getCurrentPosition()+leftDrive_0.getCurrentPosition()+
                    leftDrive_1.getCurrentPosition()/4)/11);
                 }
                telemetry.addData("Coords:", posX+","+posY);
                telemetry.update();
            }
            while(posY != 50){
                //MoveY
                if(true)
                {
                    rightDrive_0.setPower(.5);
                    rightDrive_1.setPower(-.5);
                    leftDrive_0.setPower(-.5);
                    leftDrive_1.setPower(.5);
                    posY = ((rightDrive_0.getCurrentPosition()+
                    -rightDrive_1.getCurrentPosition()+-leftDrive_0.getCurrentPosition()+
                    leftDrive_1.getCurrentPosition()/4)/11);
                }
                telemetry.addData("Coords:", posX+","+posY);
                telemetry.update();
            }
            rightDrive_0.setPower(0);
            rightDrive_1.setPower(0);
            leftDrive_0.setPower(0);
            leftDrive_1.setPower(0);
        }
        */
    }
}
