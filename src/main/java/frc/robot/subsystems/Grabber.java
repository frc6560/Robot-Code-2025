package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ResetMode;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.core.CoreCANrange;



public class Grabber extends SubsystemBase {

    private final SparkMax GMOTOR;
    private final CoreCANrange CANrangedistance;
    private final double DISTANCE_THRESHOLD = 1.0;     
    public Grabber() {
    this.GMOTOR = new SparkMax(Constants.GrabberConstants.GRABBER_MOTOR_ID, SparkMax.MostorType.kBrushless);

    this.CANrangedistance = new CoreCANrange(1); // Use the actual CAN ID of your sensor

    }

    public void grab() {
        double distance = CANrangedistance.getDistance(); // Get the current distance (in inches 

    if (distance > DISTANCE_THRESHOLD) {
        GMOTOR.set(0.1);
    } else {    
        stop();
    }
}
    public void release() {
        GMOTOR.set(-0.1);
    }
    public void stop() {
        GMOTOR.set(0);
    }
   
    @Override
    public void periodic() {
        grab(); // Automatically runs the grab in periodic liewke signals     
    } 
}

