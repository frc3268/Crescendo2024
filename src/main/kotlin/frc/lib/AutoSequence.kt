package frc.lib

import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import kotlinx.serialization.Serializable
import kotlinx.serialization.json.Json

@Serializable
class AutoSequence (val commands:Array<Array<String>>){
    fun toCommandGroup(commandsMap:Map<String, Command>):SequentialCommandGroup{
        var sq = SequentialCommandGroup()
        commands.mapIndexed{ _, commandList -> {
            var pcg = ParallelCommandGroup()
            commandList.mapIndexed{
                _, command -> pcg.addCommands(commandsMap[command])
            }
            sq.addCommands(pcg)
        } }
        return sq
    }
}