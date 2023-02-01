package frc.robot.subsystems;

import com.ctre.phoenix.sensors.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.util.ModuleState;
import frc.lib.util.SwerveTypeConstants;

import static frc.robot.Constants.*;

public class SwerveModule {
    public int moduleNum;
    private SwerveTypeConstants swerveTypeConstants;
    private Rotation2d angleOffSet;
    private Rotation2d lastAngle;
    private CANSparkMax driveNEO;
    private RelativeEncoder driveMotorEncoder;
    private CANSparkMax angleNEO;
    private RelativeEncoder angleMotorEncoder;
    private CANCoder angleCANCoder;

    SimpleMotorFeedforward feedforward =
            new SimpleMotorFeedforward(SWERVE_DRIVE_KS,SWERVE_DRIVE_KV,SWERVE_DRIVE_KA);

    public SwerveModule(
            int moduleNum,
            SwerveTypeConstants swerveTypeConstants,
            int driveMotorID, int angleMotorID, int canCoderID,
            Rotation2d angleOffSet){
        this.swerveTypeConstants = swerveTypeConstants;
        this.moduleNum = moduleNum;
        this.angleOffSet = angleOffSet;

        angleCANCoder = new CANCoder(canCoderID);
        configAngleCanCoder();

        driveNEO = new CANSparkMax(driveMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        configDriveMotor();
        driveMotorEncoder = driveNEO.getEncoder();
        configDriveMotorEncoder();


        angleNEO = new CANSparkMax(angleMotorID, CANSparkMaxLowLevel.MotorType.kBrushless);
        configAngleMotor();
        angleMotorEncoder = angleNEO.getEncoder();
        configAngleMotorEncoder();

        lastAngle = getState().angle;
    }
    public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop){
        // Custom optimize command, since default WPILib optimize assumes continuous controller which
        // REV and CTRE are not
        desiredState = ModuleState.optimize(desiredState, getState().angle);

        setAngle(desiredState);
        setSpeed(desiredState, isOpenLoop);
    }

    private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop){
        if(isOpenLoop){
            double percentOutput = desiredState.speedMetersPerSecond / SWERVE_MAX_SPEED;
            driveNEO.set(percentOutput);
        }
        else {
            driveNEO.getPIDController().setReference(
                    desiredState.speedMetersPerSecond,
                    CANSparkMax.ControlType.kVelocity,
                    0,
                    feedforward.calculate(desiredState.speedMetersPerSecond));
        }
    }

    private void setAngle(SwerveModuleState desiredState){
        Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SWERVE_MAX_SPEED * 0.01)) ? lastAngle : desiredState.angle; //Prevent rotating module if speed is less then 1%. Prevents Jittering.
        angleNEO.getPIDController().setReference(angle.getDegrees(), CANSparkMax.ControlType.kPosition);
        lastAngle = angle;
    }

    private Rotation2d getAngle(){
        return Rotation2d.fromDegrees(angleMotorEncoder.getPosition());
    }

    public Rotation2d getCanCoder(){
        return Rotation2d.fromDegrees(angleCANCoder.getAbsolutePosition());
    }

    public void resetToAbsolute(){
        double absolutePosition = getCanCoder().getDegrees() - angleOffSet.getDegrees();
        angleMotorEncoder.setPosition(absolutePosition);
    }


    private void configDriveMotor(){
        driveNEO.restoreFactoryDefaults();
        //Set drive motor config.
        driveNEO.getPIDController().setP(SWERVE_DRIVE_MOTOR_KP);
        driveNEO.getPIDController().setI(SWERVE_DRIVE_MOTOR_KI);
        driveNEO.getPIDController().setD(SWERVE_DRIVE_MOTOR_KD);
        driveNEO.getPIDController().setFF(SWERVE_DRIVE_MOTOR_KF);
        driveNEO.setSmartCurrentLimit(SWERVE_DRIVE_CURRENT_LIMIT);
        driveNEO.setOpenLoopRampRate(SWERVE_DRIVE_MOTOR_OPENLOOPRAMP);
        driveNEO.setClosedLoopRampRate(SWERVE_DRIVE_MOTOR_CLOSELOOPRAMP);
        driveNEO.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        driveNEO.setInverted(swerveTypeConstants.driveMotorInvert);
        driveNEO.setIdleMode(DRIVE_NEUTRAL_MODE);
        driveNEO.burnFlash();
    }

    private void configDriveMotorEncoder(){
        driveMotorEncoder.setVelocityConversionFactor(SWERVE_WHEEL_CIRCUMFERENCE / swerveTypeConstants.driveGearRatio / 60.0);
        driveMotorEncoder.setPositionConversionFactor(SWERVE_WHEEL_CIRCUMFERENCE / swerveTypeConstants.driveGearRatio);
        driveMotorEncoder.setPosition(0);
    }

    private void configAngleMotor(){
        angleNEO.restoreFactoryDefaults();
        //Set drive motor config.
        angleNEO.getPIDController().setP(swerveTypeConstants.angleKP);
        angleNEO.getPIDController().setI(swerveTypeConstants.angleKI);
        angleNEO.getPIDController().setD(swerveTypeConstants.angleKD);
        angleNEO.getPIDController().setFF(swerveTypeConstants.angleKF);
        angleNEO.setSmartCurrentLimit(SWERVE_ANGLE_CURRENT_LIMIT);
        angleNEO.enableVoltageCompensation(VOLTAGE_COMPENSATION);
        angleNEO.setInverted(swerveTypeConstants.angleMotorInvert);
        angleNEO.setIdleMode(ANGLE_NEUTRAL_MODE);
        angleNEO.burnFlash();
    }

    private void configAngleMotorEncoder(){
        angleMotorEncoder.setPositionConversionFactor(360 / swerveTypeConstants.angleGearRatio);
        resetToAbsolute();
    }

    private void configAngleCanCoder(){
        CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
        canCoderConfiguration.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        canCoderConfiguration.sensorDirection = swerveTypeConstants.canCoderInvert;
        canCoderConfiguration.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        canCoderConfiguration.sensorTimeBase = SensorTimeBase.PerSecond;

        angleCANCoder.configFactoryDefault();
        angleCANCoder.configAllSettings(canCoderConfiguration);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(driveMotorEncoder.getVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
                driveMotorEncoder.getVelocity() * SWERVE_WHEEL_CIRCUMFERENCE,
                getAngle());
    }
}
