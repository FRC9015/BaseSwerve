package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.math.Conversions;
import frc.lib.util.CTREModuleState;
import frc.lib.util.SwerveModuleConstants;


public class SwerveModule {
    public final int moduleNumber;
    private Rotation2d angleOffset;
    private Rotation2d latestAngle;

    private CANSparkMax mAngleMotor;
    private CANSparkMax mDriveMotor;
    public CANCoder angleEncoder;

    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);

    public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants){
        this.moduleNumber = moduleNumber;
        this.angleOffset = moduleConstants.angleOffset;
        
        /* Angle Encoder Config */
        angleEncoder = new CANCoder(moduleConstants.cancoderID);
        configAngleEncoder();
        
        /* Angle Motor Config */
        mAngleMotor = new CANSparkMax(moduleConstants.angleMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        configAngleMotor();

        /* Drive Motor Config */
        mDriveMotor = new CANSparkMax(moduleConstants.driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        configDriveMotor();

        latestAngle = getState().angle;
    }

    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        /* This is a custom optimize function, since default WPILib optimize assumes continuous controller which CTRE and Rev onboard is not */
        desiredState = CTREModuleState.optimize(desiredState, getState().angle); 
        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
            mDriveMotor.set(percentOutput);
        }
        else {
            double velocity = Conversions.MPSToFalcon(desiredState.speedMetersPerSecond, Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio);
            mDriveMotor.getPIDController().setReference(velocity, CANSparkMax.ControlType.kVelocity, 0, feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01)) ? latestAngle : desiredState.angle; //Prevent rotating module if speed is less than 1%. Prevents Jittering.
        
        mAngleMotor.getPIDController().setReference(Conversions.degreesToFalcon(angle.getDegrees(), Constants.Swerve.angleGearRatio), CANSparkMax.ControlType.kPosition);
        latestAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(Conversions.falconToDegrees(mAngleMotor.getEncoder().getPosition(), Constants.Swerve.angleGearRatio));
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleEncoder.getPosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = Conversions.degreesToFalcon(getCanCoder().getDegrees() - angleOffset.getDegrees(), Constants.Swerve.angleGearRatio);
        mAngleMotor.getEncoder().setPosition(absolutePosition);
    }

    private void configAngleEncoder(){        
        angleEncoder.configFactoryDefault();  // Adjust if needed
        angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);  // Adjust if needed
    }

    private void configAngleMotor(){
        mAngleMotor.restoreFactoryDefaults();
        mAngleMotor.setInverted(Constants.Swerve.angleMotorInvert);
        mAngleMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
        resetToAbsolute();
    }

    private void configDriveMotor(){        
        mDriveMotor.restoreFactoryDefaults();
        mDriveMotor.setInverted(Constants.Swerve.driveMotorInvert);
        mDriveMotor.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(
            Conversions.falconToMPS(mDriveMotor.getEncoder().getVelocity(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        ); 
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            Conversions.falconToMeters(mDriveMotor.getEncoder().getPosition(), Constants.Swerve.wheelCircumference, Constants.Swerve.driveGearRatio), 
            getAngle()
        );
    }
}
