package frc.robot.LimeLight;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LimeConstants;

/**
 * Subsystem para controlar y obtener datos de la LimeLight.
 * Proporciona métodos para acceder a la información de la cámara, como el Yaw,
 * TY (desplazamiento vertical), la ID de una AprilTag y calcular distancias.
 * También permite configurar el pipeline y controlar los LEDs de la cámara.
 * 
 * @author Juan Felipe Zepeda del Toro
 * @author Fernando Joel Cruz Briones
 * @version 1.1
 */
public class LimeLightSubsystem extends SubsystemBase {

    // Tabla de NetworkTables para la LimeLight
    private final NetworkTable limeLightTable;

    /**
     * Constructor: Inicializa la LimeLight y configura el pipeline predeterminado.
     */
    public LimeLightSubsystem() {
        limeLightTable = NetworkTableInstance.getDefault().getTable("limelight");
        limeLightTable.getEntry("pipeline").setNumber(0); // Configura el pipeline por defecto
    }
    
    /**
     * Actualiza periódicamente los valores de la LimeLight en el SmartDashboard.
     */
    @Override
    public void periodic() {        
        SmartDashboard.putNumber("Target ID", getID());
        SmartDashboard.putNumber("Target TX", getYaw());
        SmartDashboard.putNumber("Target TY", getTY());
        SmartDashboard.putNumber("Speaker Distance", getDistance(LimeConstants.kTargetSpeakertHeight));
        SmartDashboard.putNumber("Speaker Error", getDistanceError(LimeConstants.kTargetSpeakertHeight, LimeConstants.kDistanceToSpeaker));
        SmartDashboard.putNumber("Amp Distance", getDistance(LimeConstants.kTargetAmpHeight));
        SmartDashboard.putNumber("Amp Error", getDistanceError(LimeConstants.kTargetAmpHeight, LimeConstants.kDistanceToAmp));
    }

    /**
     * Obtiene el desplazamiento horizontal (Yaw) hacia el objetivo.
     * 
     * @return El valor de TX (Yaw) en grados.
     */
    public double getYaw() {
        return limeLightTable.getEntry("tx").getDouble(0);
    }
    
    /**
     * Obtiene el desplazamiento vertical (TY) hacia el objetivo.
     * 
     * @return El valor de TY (desplazamiento vertical) en grados.
     */
    public double getTY() {
        return limeLightTable.getEntry("ty").getDouble(0);
    }
    
    /**
     * Obtiene el ID de la AprilTag detectada.
     * 
     * @return El ID de la AprilTag como un número entero.
     */
    public double getID() {
        return limeLightTable.getEntry("tid").getDouble(0);
    }

    /**
     * Calcula la distancia horizontal entre el robot y el objetivo.
     * 
     * @param targetHeight Altura del objetivo (en pulgadas o metros).
     * @return La distancia horizontal calculada entre el robot y el objetivo.
     */
    public double getDistance(double targetHeight) { 
        double targetOffsetAngle_Vertical = getTY();
        return (targetHeight - LimeConstants.cameraHeight) 
                / Math.tan(Math.toRadians(LimeConstants.cameraAngle + targetOffsetAngle_Vertical));
    }

    /**
     * Calcula el error de la distancia entre la posición actual y la posición deseada.
     * 
     * @param height Altura del objetivo (en pulgadas o metros).
     * @param distance Distancia objetivo ideal al objetivo.
     * @return El error de distancia (positivo o negativo).
     */
    public double getDistanceError(double height, double distance) {
        return (getDistance(height) - distance);
    }

    /**
     * Comprueba si el robot está alineado con el objetivo.
     * 
     * @param height Altura del objetivo.
     * @param distance Distancia objetivo ideal.
     * @param dE Margen de error permitido para la distancia.
     * @param gY Margen de error permitido para el Yaw.
     * @return True si el robot está alineado dentro del margen especificado; false de lo contrario.
     */
    public boolean isTargetAligned(double height, double distance, double dE, double gY) {
        return Math.abs(getDistanceError(height, distance)) < dE && Math.abs(getYaw()) < gY;
    }

    /**
     * Configura el pipeline activo de la LimeLight.
     * 
     * @param pipeline El número del pipeline deseado (0-9).
     */
    public void setPipeline(int pipeline) {
        limeLightTable.getEntry("pipeline").setNumber(pipeline);
    }

    /**
     * Configura el modo de los LEDs de la LimeLight.
     * 
     * @param mode Modo de los LEDs:
     *             0: Encendidos sólidos.
     *             1: Apagados.
     *             2: Parpadeando.
     */
    public void setLimeLed(int mode) {
        limeLightTable.getEntry("ledMode").setNumber(mode);
    }
}
