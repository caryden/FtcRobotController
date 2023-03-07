package org.firstinspires.ftc.teamcode.tests.commands

//import io.mockk.mockk
import org.firstinspires.ftc.teamcode.commands.TestDrive
import org.firstinspires.ftc.teamcode.subsystems.LeapfrogDriveBase
import org.junit.Assert.*

import org.junit.After
import org.junit.Assert
import org.junit.Before
import org.junit.Test
import org.mockito.Mockito
import org.mockito.kotlin.verify
import java.util.function.DoubleSupplier

class TestDriveTests {

    @Before
    fun setUp() {
    }

    @After
    fun tearDown() {
    }

    @Test
    fun execute() {
        val forwardValue = 0.5;
        val leapfrogDriveBase = Mockito.mock<LeapfrogDriveBase>()
        val testDrive = TestDrive(leapfrogDriveBase, DoubleSupplier { forwardValue })
        testDrive.execute()
        verify(leapfrogDriveBase).drive(forwardValue)
    }

}