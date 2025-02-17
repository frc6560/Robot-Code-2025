package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;

import frc.robot.Constants;
import frc.robot.ManualControls;

import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.ClimbStates;
public class Climb extends SubsystemBase {
    
    // motors are linked up in reverse
    private TalonFX motor1;
    private TalonFX motor2;

    // encoder moves in same direction as motor1
    private CANcoder absoluteEncoder;
    private double initialEncoderPos;
    private TalonFXConfiguration fxConfig;

    public enum State {
        DOWN,
        UP,
        MOVING
    }

    private State state;

    private double targetPos = 0;

    private final NetworkTable ntTable = NetworkTableInstance.getDefault().getTable("Climb");
    private final NetworkTableEntry ntAngle = ntTable.getEntry("Angle");
    private final NetworkTableEntry ntPosition = ntTable.getEntry("Climb position");
    private final NetworkTableEntry ntTargetPos = ntTable.getEntry("Target angle");

    public Climb(ManualControls controls) { 

        this.motor1 = new TalonFX(ClimbConstants.MOTOR_1_ID, "Canivore");
        this.motor2 = new TalonFX(ClimbConstants.MOTOR_2_ID, "Canivore");
        this.absoluteEncoder = new CANcoder(ClimbConstants.CANCODER_ID, "Canivore");

        // adjust this
        this.motor1.setPosition(0);
        this.motor2.setPosition(0);
        initialEncoderPos = 0;

        this.fxConfig = new TalonFXConfiguration(); 
        fxConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        motor1.getConfigurator().apply(fxConfig);
        motor2.getConfigurator().apply(fxConfig);

        // deprecated but whatever
        motor2.setInverted(true);

        Slot0Configs climbPIDcontroller = new Slot0Configs();
        
        // placeholder values adjust
        climbPIDcontroller.kS = 0;
        climbPIDcontroller.kG = 0;
        climbPIDcontroller.kP = 1.2;
        climbPIDcontroller.kI = 0.014;
        climbPIDcontroller.kD = 0;       

        motor1.getConfigurator().apply(climbPIDcontroller);
        motor2.getConfigurator().apply(climbPIDcontroller);

        ntAngle.setDouble(0.0);
        ntTargetPos.setDouble(this.targetPos);
        ntPosition.setDouble(motor1.getPosition().getValueAsDouble());
    }

    @Override
    public void periodic() {
        ntAngle.setDouble(getClimbAngle());
        ntTargetPos.setDouble(this.targetPos);
        ntPosition.setDouble(motor1.getPosition().getValueAsDouble());
    }

    public double getClimbAngle() {
        return absoluteEncoder.getPosition().getValueAsDouble() * 360 / ClimbConstants.GEAR_RATIO;
    }

    // raw motor pos
    public double getClimbMotorPosition() {
        return motor1.getPosition().getValueAsDouble();
    }

    public double getClimbAngularVelocity() {
        return absoluteEncoder.getVelocity().getValueAsDouble() * 360 / ClimbConstants.GEAR_RATIO;
    }

    public double getUpperBound() {
        return ClimbConstants.UPPER_SOFT_BOUND;
    }

    public double getLowerBound() {
        return ClimbConstants.LOWER_SOFT_BOUND;
    }

    public boolean getOvershoot() {
        double position = this.getClimbAngle();
        return (position > ClimbConstants.UPPER_SOFT_BOUND || position < ClimbConstants.LOWER_SOFT_BOUND);
    }

    public State getState() {
        if (getClimbAngularVelocity() > 0.1) {
            this.state = State.MOVING;

        } else {

            double angle = getClimbAngle();

            if (Math.abs(angle - ClimbStates.UP) < ClimbConstants.TOLERANCE) {
                this.state = State.DOWN;
            } else if (Math.abs(angle - ClimbStates.DOWN) < ClimbConstants.TOLERANCE){
                this.state = State.UP;
            } else {
                this.state = State.MOVING;
            }
        }
        return this.state;
    }

    // sets odometry readings, doesn't actually move anything
    public void setEncoderPosition(double pos) {
        this.absoluteEncoder.setPosition(pos);
        this.motor1.setPosition(pos);
        this.motor2.setPosition(pos);
    }

    // actually moves climb
    public void setClimbPosition(double pos) {
        pos = Math.min(Math.max(pos, ClimbConstants.LOWER_SOFT_BOUND), ClimbConstants.UPPER_SOFT_BOUND);
        this.targetPos = pos;
        double targetMotorPos = pos / 360 * ClimbConstants.GEAR_RATIO;
        final PositionVoltage m_request = new PositionVoltage(targetMotorPos);
        motor1.setControl(m_request);
        motor2.setControl(m_request);
    }

    public void stopMotor() {
        this.motor1.set(0);
        this.motor2.set(0);
    }

    // for testing only
    public void testMotors() {
        this.motor1.set(0.1);
        this.motor2.set(0.1);
    }
}

