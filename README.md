# EFU
Software for the FLOATS End of Fiber Unit


FLOATS End of Fiber Unit Procedures and Operation
Doug Goetz August 2019

Operating Modes:

1.	Startup and Reset Mode: This mode loads the startup configuration when the EFU is initially powered on or when reset when the watchdog timer is reset. The initial configuration is to turn off the charger board (solar shutdown engaged, charger off) to prevent damage to the board and to power off: Ublox GPS (DIO pin 4  = low), Fiber Optic Transmitter (DIO pin 13 = low). The LTC Temperature chip will be configured to monitor internal temperatures during startup. If the temperature is too low for charging and operations the heater will power on and the system will wait until up to temperature. The 3.7V rechargeable battery voltage will also be monitored and recharged if the voltage is too low to begin normal operations. This isn’t necessary during normal start up in flight, but might be necessary after watchdog reset. Once T and V are ok, transition into warmup mode.

2.	Warmup and RTC Set Mode: In this mode the GPS turned on and checked for a lock. Once locked the RTC will be set based on GPS time. Once the RTC is set a flag will determine if the EFU goes into a standby mode or measurement mode. The flag is based on what the mode was prior to warmup. If it was previously in startup, measurement, or telemetry mode it will transition to standby mode. If it was in standby, it will transition to measurement mode.

3.	Standby Mode: This mode is used for housekeeping operations or as a placeholder for a “low power/sleep” state if needed.

4.	Measurement Mode: Here the GPS will be powered on and warmed up, and the RTC will be set prior to entering the mode. The LTC temperature chip will also be on. GPS position, temperature of all sensors, analog voltages, and time will be recorded at low frequency (0.1 Hz?) until the telemetry alarm has been triggered (or if a measurement timer is needed and has concluded). The recorded data will be averaged and put into a buffer that has been populating for that hour. When/if kicked out of measurement mode and standby is needed, the measurement flag will be set to 0.

5.	Telemetry Mode: A minute alarm every hour will be used to interrupt into telemetry mode from any other mode. In this mode everything but the Fib Tx will be powered off (including solar charger). The telemetry will be sent for a period equal to ttx (around 1 minute). Once the period expires the EFU will go into warmup mode.


Timing:
Reset/Startup: in this state until conditions are safe to power everything on
Warmup: in this state until a valid GPS fix is made after reset
Measurement: 20s measurements based on millis() timer; after 1 minute the measurements are averaged and placed in telemetry buffer
Telemetry: 1Hz transmission (millis() timer) over a one minute period (RTC?)


Functions:
ManageState():Switch-Case that uses EFUState Enum to control state machine. All operational functions (e.g. GPSon(), Solaroff()) are held inside this function. The timer function TimeManager() uses the same Enum to control when the state machine changes. To do: implement watchdog timer for each state, record T,GPS and HK measurements to temporary buffer 

TimeManager(): Switch-Case that uses EFUState Enum to control timing of state and switching between states. Millis() and RTC minute timers are used. Variables for the timers are contained in the .cpp file as ints. XXXXPeriod variables are used to determine the length of the state. XXXXInterval variables are used for setting the state loop timing. To do: implement an averaging function to be used during when the MeasurePeriod is over. 

ResetConfiguration(): Function used during startup or after reset to set the EFU into a safe configuration. (e.g. solar panel enable off)

HeaterControl(int setT, int setTspan): Function that used the LTC temperature measurement of the batteries to control turn on/off the battery heater. To do: error checking and reset when -999. Include a timer or other backup for if the LTC fails. Include board thermistor in the heat control.

ConfigureChannels: configures LTC2983 chip. To do: include custom coefficients for board thermistor



Heater: 
10.6ohms @3.6V = 1.21 watts
5 ohms @3.6V = 2.6 watts
8.2 ohms @3.6V = ~1.6 watts

Mass:
Solar panel x1 = 27g
Solar panel x4 = 108g
EFU test box with electronics and batteries = 536g


