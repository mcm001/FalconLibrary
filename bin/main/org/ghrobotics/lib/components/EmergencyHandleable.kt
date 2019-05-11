package org.ghrobotics.lib.components

@Suppress("unused")
interface EmergencyHandleable {

    sealed class Severity {

        object Nothing : Severity()

        object DisableClosedLoop : Severity()

        object DisableOutput : Severity()

    }

//    @JvmDefault
    fun activateEmergency(severity: Severity)// {
//        when (severity) {
//            is Severity.DisableClosedLoop -> disableClosedLoop()
//            is Severity.DisableOutput -> disableOutputs()
//        }
//    }

//    fun disableClosedLoop()

//    fun disableOutputs()

    fun recoverFromEmergency()
}