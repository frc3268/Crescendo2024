package frc.lib

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.button.Trigger

enum class TriggerType { ON_TRUE, WHILE_TRUE, TOGGLE}

fun AutoCommand(command: Command, autoMap: MutableMap<String, Command>? = null, binding: Trigger? = null, name:String? = null, shuffleboardTab: ShuffleboardTab?  = null, triggerType: TriggerType = TriggerType.ON_TRUE) {

    if(name != null){
        if(autoMap != null){
            autoMap[name] = command
        }
        shuffleboardTab?.add(name, command)

    }
    when(triggerType){
        TriggerType.ON_TRUE ->
            binding?.onTrue(command)
        TriggerType.WHILE_TRUE ->
            binding?.whileTrue(command)
        TriggerType.TOGGLE ->
            binding?.toggleOnTrue(command)
    }
}