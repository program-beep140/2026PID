package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkRelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Turret2Subsystem extends SubsystemBase {
    //private final VictorSP armMotor;
    //private final Encoder armEncoder;
    //private final ProfiledPIDController armController;

    // PID Gains and Motion Profile Constraints
    private static double kP = 0.5; // Proportional gain
    private static double kI = 0.0001; // Integral gain
    private static double kD = 0.0; // Derivative gain
    private static double maxOutput = 0.3;
    private static final double kMaxVelocity = 1.0; // Max velocity in units/sec
    private static final double kMaxAcceleration = 0.5; // Max acceleration in units/sec^2
    private static final double kTurretGearRatio = 1.00/15.00;// 15 dev of moter is one rev of turret

    private SparkMax m_motor = new SparkMax(Constants.TurretConstants.TURRET_MOTOR_ID, MotorType.kBrushless);
    //public SparkAbsoluteEncoder absAngleEncoder = m_motor.getAbsoluteEncoder();
       
    //private SparkMaxConfig m_config = new SparkMaxConfig();
    private SparkMaxConfig m_baseConfig = new SparkMaxConfig();
    private SparkClosedLoopController closedLoopController = m_motor.getClosedLoopController();
    private RelativeEncoder turretEncoder;
    
    private double angleSetpoint = 0;

    public Turret2Subsystem() 
    {
        // angleSetpoint = absAngleEncoder.getPosition(); //Gets position in rotations        

        turretEncoder = m_motor.getEncoder();
        turretEncoder.setPosition(angleSetpoint); //update the position on motor encoder
        
        m_baseConfig.closedLoop
                        .p(kP)
                        .i(kI)
                        .d(kD)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange((-1 * maxOutput),maxOutput); // set PID and 1/3 max speeds                        
        m_baseConfig.idleMode(IdleMode.kBrake);
        m_baseConfig.encoder.positionConversionFactor(kTurretGearRatio);

        //Update the motoro config to use PID
        m_motor.configure(m_baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

            // Initialize dashboard values
        SmartDashboard.setDefaultNumber("Turret Target Position", 0);
        SmartDashboard.setDefaultNumber("Turret Target Velocity", 0);
        SmartDashboard.setDefaultBoolean("Turret GO", false);
        SmartDashboard.setDefaultBoolean("Turret Reset Encoder", false);
        SmartDashboard.setDefaultNumber("Turret Relative Angle", 0);
        SmartDashboard.putNumber("Turret P Gain", kP);
        SmartDashboard.putNumber("Turret I Gain", kI);
        SmartDashboard.putNumber("Turret D Gain", kD);
        SmartDashboard.putNumber("Turret IAccum", 0);
        SmartDashboard.setDefaultBoolean("Turret Stop", false);

        Shuffleboard.getTab("Turret Sysid Testing").addDouble("Turret Relative Angle", turretEncoder::getPosition);
        Shuffleboard.getTab("Turret Sysid Testing").addDouble("Turret Angle ProfileGoal", () -> angleSetpoint);
        Shuffleboard.getTab("Turret Sysid Testing").addDouble("Turret Angle Motor Current", m_motor::getOutputCurrent);        
        Shuffleboard.getTab("Turret Sysid Testing").addDouble("Turret Angle Motor Output", m_motor::getAppliedOutput);
    }

    
    public void setAngleMotor(double speed)
    {
        if (speed > 1) speed = 1;
        else if (speed < -1) speed = -1;
        m_motor.setVoltage(1 * speed);
    }

    /**
     * Stops the motor
     */
    public void stop()
    {
        //set the current
        double currentAngle = getTurretDegrees(turretEncoder.getPosition());
        closedLoopController.setSetpoint(currentAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        closedLoopController.setIAccum(0);
        //turn off motor
        setAngleMotor(0);

        //turn off the control mode
        SmartDashboard.putBoolean("Turret GO", false);
    }

    /**
     * This will convert the motors rotations into a 360 degree for the turret
     * @param motorRotations
     * @return
     */
    private double getTurretDegrees(double motorRotations)
    {
        //Convert the motor revs using ratio
        //double adjustedRotations = motorRotations * kTurretGearRatio;
        //done in the controller now
        double adjustedRotations = motorRotations;

        return Units.rotationsToDegrees(adjustedRotations);
    }

    private double getTurretRotations(double turretDegrees)
    {
        //get normal rotations from degrees
        double rotations = Units.degreesToRotations(turretDegrees);
        //Done in the controller now
        //rotations = rotations * (1 / kTurretGearRatio);
        return rotations;
    }
    

    public void periodic() 
    {    
        double currentMotorRotations = turretEncoder.getPosition();

        // Display encoder position and velocity
        SmartDashboard.putNumber("turret Actual Position", currentMotorRotations);
        SmartDashboard.putNumber("turret Actual Velocity", turretEncoder.getVelocity());
        double currentAngleRot2Degree = getTurretDegrees(currentMotorRotations);
        SmartDashboard.putNumber("turret Relative Angle rot2deg", currentAngleRot2Degree);
        SmartDashboard.putNumber("turret IAccum", closedLoopController.getIAccum());

        if (SmartDashboard.getBoolean("turret Reset Encoder", false)) {
            SmartDashboard.putBoolean("turret Reset Encoder", false);
            // Reset the encoder position to abs value
            turretEncoder.setPosition(0);
            SmartDashboard.putNumber("Target Position",0);
        }
        if (SmartDashboard.getBoolean("Turret Stop", false)) 
        {
            SmartDashboard.putBoolean("Turret Stop", false);
            //Turn off motor
            stop();
        }
        else if (SmartDashboard.getBoolean("Turret GO", false)) 
        {
            SmartDashboard.putBoolean("Turret GO", false);
            /*
            * Get the target position from SmartDashboard and set it as the setpoint
            * for the closed loop controller.
            */
            double targetPosition = SmartDashboard.getNumber("Target Position", 0);
            if (targetPosition > 360) {                
                targetPosition = 360;
                SmartDashboard.putNumber("Target Position",targetPosition);
            } else if (targetPosition < 0) {
                targetPosition = 0;
                SmartDashboard.putNumber("Target Position",targetPosition);
            }
            //TODO: get current position in degrees, find how many times around we are, -1,-2, 2, etc
                
            double targetPositionRotations = getTurretRotations(targetPosition);

            //Read PID values
            // read PID coefficients from SmartDashboard
            double p = SmartDashboard.getNumber("Turret P Gain", 0);
            double i = SmartDashboard.getNumber("Turret I Gain", 0);
            double d = SmartDashboard.getNumber("Turret D Gain", 0);
            
            // if PID coefficients on SmartDashboard have changed, write new values to controller
            boolean updatePID = false;
            if((p != kP)) { updatePID = true;  kP = p; }
            if((i != kI)) { updatePID = true;  kI = i; }
            if((d != kD)) { updatePID = true; kD = d; }

            if (updatePID)
            {
                //Update the PID on close loopController
                m_baseConfig.closedLoop
                        .p(kP)
                        .i(kI)
                        .d(kD)
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .outputRange((-1 * maxOutput),maxOutput); // set PID and 1/3 max speeds
                //Update the motoro config to use PID
                m_motor.configure(m_baseConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
            } //end if updatePID

            //Run the motor
            closedLoopController.setSetpoint(targetPositionRotations, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
    }

 
}