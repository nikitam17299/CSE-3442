# CSE-3442
Embedded Systems Labs and Final Project

The labs were built in preparation for the final project. 

Final Project - An embedded systems project to build a device that serves as an actuator for a door lock with a programmable IR remote control, lock status monitoring, and local announciators using Tivaware Microcontroller. The software supported UART commands on virtual COM port that delivers instructions such as lock, unlock, status of the lock and listens for a sequence of remote-control presses that acts as interrupts and then sets password. After 10 seconds, the entry times out and the sequence is memorized in EEPROM. Passwords can be erased and a panic signal can be set which in turn sets off an alarm on a speaker(using PWM). The software was also configured to set time of the lock and also set a time that the actuator should automatically lock/unlock utilizing the RTC module. 
