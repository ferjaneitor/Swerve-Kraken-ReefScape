package frc.robot.Intakes.Coral;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.CoralConstants;

/**
 * CoralPivotCmd es un comando continuo que controla el ángulo o pivote del mecanismo
 * del subsistema Coral. Ajusta la velocidad del pivote en función de CoralConstants.CoralPivotMaxVelocity
 * y de la bandera DirectionInverted.
 * 
 * Funcionamiento:
 * - initialize(): No realiza ninguna acción.
 * - execute(): Calcula la velocidad final multiplicando el valor obtenido del joystick (yJoystickSupplier)
 *   por CoralConstants.CoralPivotMaxVelocity y ajusta la dirección según DirectionInverted. Si el valor
 *   del joystick es mayor o igual a 0.2 o menor o igual a -0.2, se activa el pivote con la velocidad
 *   calculada; de lo contrario, se detiene el pivote.
 * - end(): Detiene el intake del mecanismo Coral llamando a stopCoralIntake().
 * - isFinished(): Retorna siempre false, por lo que el comando no finaliza de forma autónoma.
 * 
 * @Autor: Fernando Joel Cruz Briones
 * @Versión: 1.1
 */
public class CoralPivotCmd extends Command {

    /**
     * Indica si se invierte la dirección del motor para el pivote.
     */
    private boolean DirectionInverted;

    /**
     * Velocidad final calculada para el pivote.
     */
    private double finalVelocity;

    /**
     * Subsistema responsable del control del pivote en el mecanismo Coral.
     */
    private final CoralSubSystem coralSubSystem;

    /**
     * Proveedor del valor del joystick en el eje Y.
     */
    private Supplier<Double> yJoystickSupplier;
    
    /**
     * Construye un nuevo comando para controlar el pivote del mecanismo Coral.
     *
     * @param invertDirection Indica si la dirección del pivote debe invertirse.
     * @param coralSubSystem  Instancia del subsistema Coral.
     * @param yJoystickSupplier Proveedor del valor del joystick en el eje Y.
     */
    public CoralPivotCmd(boolean invertDirection, CoralSubSystem coralSubSystem, Supplier<Double> yJoystickSupplier) {
        this.DirectionInverted = invertDirection;
        this.coralSubSystem = coralSubSystem;
        this.yJoystickSupplier = yJoystickSupplier;
        addRequirements(coralSubSystem);
    }

    /**
     * Inicializa el comando. No se realiza ninguna acción adicional.
     */
    @Override
    public void initialize() {
        // No se requiere inicialización.
    }

    /**
     * Se llama repetidamente mientras el comando está activo. Calcula y aplica la velocidad
     * al pivote del mecanismo Coral según el valor del joystick.
     */
    @Override
    public void execute() {
        finalVelocity = (yJoystickSupplier.get() * CoralConstants.CoralPivotMaxVelocity)
                        * (DirectionInverted ? -1 : 1);
        
        if (yJoystickSupplier.get() >= 0.2 || yJoystickSupplier.get() <= -0.2) {
            coralSubSystem.enablePivot(finalVelocity);
        } else {
            coralSubSystem.stopPivot();
        }
    }

    /**
     * Se llama al finalizar o interrumpir el comando, deteniendo el intake del mecanismo Coral.
     *
     * @param interrupted Indica si el comando terminó de forma normal o fue interrumpido.
     */
    @Override
    public void end(boolean interrupted) {
        coralSubSystem.stopCoralIntake();
    }

    /**
     * Este comando nunca finaliza por sí solo.
     *
     * @return false siempre.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
