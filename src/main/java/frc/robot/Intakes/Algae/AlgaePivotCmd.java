package frc.robot.Intakes.Algae;

import java.util.function.Supplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.AlgaeConstants;

/**
 * AlgaePivotCmd es un comando continuo que controla la inclinación o pivote
 * del mecanismo de intake (Algae). Permite mover el pivote a una velocidad
 * específica y en una dirección determinada mediante la bandera isInverted.
 * 
 * Funcionalidades:
 * - En initialize(), se calcula la velocidad final según la constante pivotMotorVelocity
 *   (definida en AlgaeConstants) y el estado de isInverted.
 * - En execute(), se llama al subsistema para aplicar la velocidad calculada al pivote.
 * - En end(), se detiene el motor del pivote.
 * - El método isFinished() retorna siempre false, por lo que este comando no termina
 *   automáticamente, sino que normalmente se interrumpe desde otro comando o evento.
 * 
 * @Autor: Fernando Joel Cruz Briones
 * @Versión: 1.1
 */
public class AlgaePivotCmd extends Command {

    /**
     * Indica si la rotación del pivote se invierte.
     */
    private boolean isInverted;

    /**
     * Subsistema que controla el mecanismo Algea (intake) y sus partes móviles, incluido el pivote.
     */
    private AlgaeSubSystem algeaSubSystem;

    /**
     * Velocidad final calculada para el pivote, basada en la constante pivotMotorVelocity y la bandera isInverted.
     */
    private double finalVelocity;

    /**
     * Proveedor que devuelve el valor del joystick en el eje Y.
     */
    private Supplier<Double> yJoystickSupplier;

    /**
     * Crea un nuevo comando para ajustar la inclinación/pivote de Algea.
     *
     * @param invertDirection  true para invertir la dirección del pivote.
     * @param algeaSubSystem   Instancia del subsistema Algea a controlar.
     * @param yJoystickSupplier Proveedor del valor del joystick en el eje Y.
     */
    public AlgaePivotCmd(boolean invertDirection, AlgaeSubSystem algeaSubSystem, Supplier<Double> yJoystickSupplier) {
        this.algeaSubSystem = algeaSubSystem;
        this.isInverted = invertDirection;
        this.yJoystickSupplier = yJoystickSupplier;
        addRequirements(algeaSubSystem);
    }

    /**
     * Inicializa el comando.
     * (No se realiza acción adicional en este método.)
     */
    @Override
    public void initialize() {
    }

    /**
     * Llamado repetidamente mientras el comando está activo.
     * Calcula y aplica la velocidad al pivote:
     * - La velocidad final se calcula multiplicando el valor del joystick por AlgaeConstants.AlgaePivotMaxVelocity
     *   y se invierte si isInverted es true.
     * - Si el valor del joystick es mayor o igual a 0.2 o menor o igual a -0.2, se activa el pivote con la velocidad calculada.
     * - En caso contrario, se detiene el pivote.
     */
    @Override
    public void execute() {
        finalVelocity = (yJoystickSupplier.get() * AlgaeConstants.AlgaePivotMaxVelocity) * (isInverted ? -1 : 1);

        if (yJoystickSupplier.get() >= 0.2 || yJoystickSupplier.get() <= -0.2) {
            algeaSubSystem.enablePivot(finalVelocity);
        } else {
            algeaSubSystem.stopPivot();
        }
    }

    /**
     * Se llama al terminar o interrumpir el comando, deteniendo el motor del pivote.
     *
     * @param interrupted Indica si el comando terminó de forma normal o fue interrumpido.
     */
    @Override
    public void end(boolean interrupted) {
        algeaSubSystem.stopPivot();
    }

    /**
     * Retorna false para indicar que este comando no termina por sí solo.
     *
     * @return false siempre.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
