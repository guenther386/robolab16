#!/usr/bin/env python3
import ev3dev.ev3 as ev3

motorA = ev3.LargeMotor('outA')
motorB = ev3.LargeMotor('outB')
#motorA.duty_cycle_sp = 0
#motorB.duty_cycle_sp = 0
motorA.command = 'reset'
motorB.command = 'reset'
