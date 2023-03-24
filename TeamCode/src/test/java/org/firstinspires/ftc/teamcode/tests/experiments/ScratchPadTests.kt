package org.firstinspires.ftc.teamcode.tests.experiments

import kotlinx.coroutines.*
import kotlinx.coroutines.channels.Channel
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.flow
import org.junit.*

class ScratchPadTests {

    @Test
    fun simple_flow_example() = runBlocking {
        // let's try to test out a kotlin StateFlow
        var flow = flow<Int>() {
            for(i in 1..10) {
                emit(i)
                delay(100)
                }
            }

        val job = GlobalScope.launch {
            flow.collect {
            println("flow: $it")
            delay(200)
           }
        }
        job.join()
    }
    @Test
    fun simple_stateflow_example() = runBlocking {
        val stateFlow = MutableStateFlow(0)

        val collector = launch { // launch a coroutine in background and continue
            stateFlow.collect {
                println("stateFlow Received: $it")
            }
        }

        val emitter = GlobalScope.launch {
            for (i in 1..10) {
                stateFlow.value = i
                delay(100)
            }
        }

        emitter.join()
        collector.cancelAndJoin()
    }
    @Test
    fun simple_channel_example() = runBlocking {

        val channel = Channel<Int>()

        val emitter = GlobalScope.launch {
            for (i in 1..10) {
                channel.send(i)
                delay(100)
            }
        }
        val collector = launch {
            while (isActive) {
                val i = channel.receive()
                println("channel Received: $i")
            }
        }

        emitter.join()
        collector.cancelAndJoin()
    }
}