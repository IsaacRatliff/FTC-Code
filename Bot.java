package workspace_;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Bot {
    private DcMotor leftDrive_0 = null;
    private DcMotor rightDrive_0 = null;
    private DcMotor leftDrive_1 = null;
    private DcMotor rightDrive_1 = null;
    private DcMotor arm = null;
    private Servo claw1 = null;
    private Servo claw2 = null;
    public final double speed = 6.45/4;  // it took 6.45 sec to go 4 tiles at full spd
    public final double turn90 = 1.8;  // 1.8 seconds to turn 90 degrees
    private ElapsedTime runtime = new ElapsedTime();
    private Telemetry telemetry = null;
    
    public void leftWheels(double spd){
        leftDrive_0.setPower(spd);
        leftDrive_1.setPower(spd);
    }
    
    public void rightWheels(double spd){
        rightDrive_0.setPower(spd);
        rightDrive_1.setPower(spd);
    }
    
    public void forward(double tiles){
        leftWheels(1.0);
        rightWheels(1.0);
        runtime.reset();
        while(runtime.seconds() < speed * tiles){
            telemetry.addData("Path", "Forward: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    
    public void backward(double tiles){
        leftWheels(-1.0);
        rightWheels(-1.0);
        runtime.reset();
        while(runtime.seconds() < speed * tiles){
            telemetry.addData("Path", "Backward: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    
    public void left(double quarters){
        leftWheels(-1.0);
        rightWheels(1.0);
        runtime.reset();
        while(runtime.seconds() < turn90 * quarters){
            telemetry.addData("Path", "Turning Left: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    
    public void right(double quarters){
        leftWheels(1.0);
        rightWheels(-1.0);
        runtime.reset();
        while(runtime.seconds() < turn90 * quarters){
            telemetry.addData("Path", "Turning Right: %2.5f S Elapsed", runtime.seconds());
            telemetry.update();
        }
    }
    
    public void stop(){
        leftWheels(0.0);
        rightWheels(0.0);
    }
    
    public Bot(HardwareMap hardwareMap, Telemetry telemetry, boolean has_arm, boolean has_claw){
        this.telemetry = telemetry;
        leftDrive_0  = hardwareMap.get(DcMotor.class, "leftDrive_0");
        rightDrive_0 = hardwareMap.get(DcMotor.class, "rightDrive_0");
        leftDrive_1  = hardwareMap.get(DcMotor.class, "leftDrive_1");
        rightDrive_1 = hardwareMap.get(DcMotor.class, "rightDrive_1");
        
        leftDrive_0.setDirection(DcMotor.Direction.FORWARD);
        rightDrive_0.setDirection(DcMotor.Direction.REVERSE);
        leftDrive_1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive_1.setDirection(DcMotor.Direction.REVERSE);
        
        if(has_arm){
            arm = hardwareMap.get(DcMotor.class, "arm");
        }
        
        if(has_claw){
            claw1 = hardwareMap.get(Servo.class, "claw1");
            claw2 = hardwareMap.get(Servo.class, "claw2");
        }
    }
    
    public Bot(HardwareMap hardwareMap, Telemetry telemetry, boolean has_arm){
        this(hardwareMap, telemetry, has_arm, has_arm);
    }
    
    public Bot(HardwareMap hardwareMap, Telemetry telemetry){
        this(hardwareMap, telemetry, true, true);
    }
}
