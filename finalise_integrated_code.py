#Created By: Lim Excelyynx, Tang Wei Shuen, Lin Yu Shun, Justin Chua Kai Yuan, Goay Kexin
#Created Date: 27/04/2025 
#version = "13.0"
'''
ENG1013: Engineering Smart System
Traffic Control System Project 2025
Tunnel Height Detection and Pedestrian Traffic Control System
-------------------------------------------------------------
Description:

This Python program runs on a PC and communicates with an Arduino board via the Pymata4 library.
Then, the scaled-down prototype will be presented on the circuit board on the demonstration session.
Timing and state transitions are handled in software using Python's time module.
This system is designed to simulate real-world traffic management using a scale model. 

The components used includes:
- Shift Registers (x3): Control multiple LEDs.
- Ultrasonic Sensors (x3): Measure distances to detect over height vehicle.
- Buzzer (PA1): Emits warning tones to emit alert signal.
- Push Button (PB1): Triggers pedestrian crossing.

The system consists of four main subsystems:

Subsystem 1 is Approach Height Detection Subsystem which handles overheight vehicle detection
            at the main tunnel entrance. An ultrasonic sensor (US1) constantly monitors for vehicles exceeding the tunnel's 
            height limit. When an overweight vehicle is detected, warning light (WL1) is turned on,
            a passive buzzer (PA1) sounds, and the traffic lights (TL1 and TL2) are set to red to prevent the vehicle from entering. 
            The system remains in this warning state until the vehicle is no longer detected,
            after which normal traffic light behavior resumes. 

Subsystem 2 is Tunnel Ave Control Subsystem which manages pedestrian crossing control.
            A pair of push button (PB1) allows pedestrians to request to cross the road.
            When pressed, traffic light (TL4) will turn red and pedestrian light (PL1) will turn green,
            then after a delay switches PL1 to flashing red to indicate time running out, and finally back to red.
            Without pressing, the pedestrian red light (PL1) remains solid. 

Subsystem 3 is Over-height Exit Subsystem which monitors the side road using a ultrasonic sensor (US3).
            When a vehicle is detected attempting to enter from this side road, 
            the system activates a dedicated traffic light (TL5) to green, 
            allowing the vehicle to merge. After a short delay, TL5 returns to red. 

Subsystem 4 Tunnel Height Detection Subsystem which detects over height vehicles the tunnel entrance
            using ultrasonic sensor (US2). When an over height vehicle is detected, 
            a pair of warning light warning light (WL2) begins flashing, 
            and the corresponding traffic light (TL3) is changed to solid red. 
            This process continues as long as the vehicle is detected, alerting the driver to avoid the tunnel.

'''

from pymata4 import pymata4
import time

board = pymata4.Pymata4()

time.sleep(2)

# set pin numbers

pb1 = 2
pb2 = 3 

us1Echo = 5 
us1Trig = 6 

us2Echo = 7 
us2Trig = 8 

us3Echo = 9 
us3Trig = 10 

serialPin1 = 11
rclkPin1 = 12
srclkPin1 = 13                                                                                           

serialPin2 = 14 #A0
rclkPin2= 15 #A1
srclkPin2 = 16 #A2                                                                                      

serialPin3 = 17 #A3
rclkPin3 = 18 #A4
srclkPin3 = 19 #A5

# the list sequence for shift register 2
redTL3 = 0 # QH
greenTL3 = 1 # QG
redWL2 = 2 # QF
greenPL1 = 3 #QE
redPL1 = 4 #QD
greenTL4 = 5 #QC
yellowTL4  = 6 #QB
redTL4 = 7 #QA
stateShiftRegister2 = [0]*8

# initial state for all boolean flag
firstTimeSub1 = False
firstTimeSub2 = False
firstTimeSub3 = False
firstTimeSub4 = False
startTime = time.time()

lastState = 0
iteration = 0
iteration1 = 0
iteration2 = 0
iteration3 = 0
iteration4 = 0
runningSub1 = False 
runningSub2 = False
runningSub3 = False 
runningSub4 = False
us3CameAndLeft = False

overheightVechicleAtUS1 = False
overheightVechicleAtUS2 = False
overheightVechicleAtUS3 = False
overheightVechicleExitedUS1 = False
overheightVechicleExitedUS2 = False
overheightVechicleExitedUS3 = False
override4I3Active = False
readyToReset4I3 = False

# set pin modes
board.set_pin_mode_digital_input(pb1)
board.set_pin_mode_pwm_output(pb2)

board.set_pin_mode_sonar(us1Trig, us1Echo)
board.set_pin_mode_sonar(us2Trig, us2Echo)
board.set_pin_mode_sonar(us3Trig, us3Echo)

board.set_pin_mode_digital_input(us1Echo)
board.set_pin_mode_digital_input(us2Echo)
board.set_pin_mode_digital_input(us3Echo)

board.set_pin_mode_digital_output(serialPin1)
board.set_pin_mode_digital_output(rclkPin1)
board.set_pin_mode_digital_output(srclkPin1)

board.set_pin_mode_digital_output(serialPin2)
board.set_pin_mode_digital_output(rclkPin2)
board.set_pin_mode_digital_output(srclkPin2)

board.set_pin_mode_digital_output(serialPin3)
board.set_pin_mode_digital_output(rclkPin3)
board.set_pin_mode_digital_output(srclkPin3)

board.digital_write(serialPin1, 0)
board.digital_write(rclkPin1, 0)
board.digital_write(srclkPin1, 0)

board.digital_write(serialPin2, 0)
board.digital_write(rclkPin2, 0)
board.digital_write(srclkPin2, 0)

board.digital_write(serialPin3, 0)
board.digital_write(rclkPin3, 0)
board.digital_write(srclkPin3, 0)

detectionThreshold = 10 # 10cm 

def main(startTime):

    """
    Main function to control the vehicle height detection and traffic management system.

    This function initializes system states and enters an infinite loop where it:
    - Monitors ultrasonic sensors for vehicle height detection.
    - Activates corresponding subsystems based on sensor input.
    - Manages override logic and reset conditions.
    - Handles subsystem-specific triggers and state transitions.
    - Supports interruption by keyboard to shut down hardware safely.

    Parameters:
    startTime (float): The system start time used for time-based logic and delays.

    Returns:
    None
    """

    try:
        global runningSub1, overheightVechicleAtUS1, overheightVechicleExitedUS1 
        global runningSub2, timePrevious2, lastState2, running3I2, startTime3I2
        global runningSub3, overheightVechicleAtUS3, overheightVechicleExitedUS3 
        global runningSub4, overheightVechicleAtUS2, overheightVechicleExitedUS2, override4I3Active
        global firstTimeSub1, firstTimeSub2, firstTimeSub3, firstTimeSub4
        
        running3I2 = False
        timePrevious2 = 0
        lastState2 = 0
        starting()
        time.sleep(1)

        print("Ready")

        while True: 
            refresh_state()
            check_reset_4I3()
            time.sleep(0.3)

            objectDistanceUS1 = get_distance(us1Trig, 1)
            objectDistanceUS2 = get_distance(us2Trig, 2)
            objectDistanceUS3 = get_distance(us3Trig, 3)

            if not runningSub1 and  not runningSub2 and not runningSub3 and not runningSub4:
                print("Call initial")
                starting()
            
            if overheightVechicleAtUS1 or runningSub1 == True:
                if not runningSub1:
                    runningSub1 = True
                    startTime1 = time.time()

                subsystem1(startTime1)  

                refresh_state()
                    
                if overheightVechicleAtUS3 or runningSub3 == True:
                    if not runningSub3:
                        runningSub3 = True 
                        startTime3 = time.time()
                  
                    subsystem1(startTime1)
                    subsystem3(startTime3)

                    refresh_state()

                    if overheightVechicleExitedUS3 and runningSub1 == True and not overheightVechicleAtUS1:
                        runningSub1 = False 
                        board.play_tone_off(pb2)
                        starting_sub1()
                        
                refresh_state()

            if overheightVechicleAtUS2:                       
                objectdistance = get_distance(us2Trig,2)
                if (objectdistance is not None and objectdistance <= detectionThreshold) or runningSub4 == True:
                    if not runningSub4:
                        runningSub4 = True
                        startTime4 = time.time()
                    subsystem4(startTime4)

            if overheightVechicleAtUS3 or runningSub3 == True:
                if not runningSub3:
                    runningSub3 = True 
                    startTime3 = time.time()
                subsystem3(startTime3)

            if (read_push_button() == True and time.time() - timePrevious2> 30 and lastState2 == 0) or runningSub2 == True: 
                if not runningSub2:
                    runningSub2 = True
                    startTime2 = time.time()
                    iteration = 0
                subsystem2(startTime2)
            
            if running3I2 == True:
                sub3i2(startTime3I2)

    except KeyboardInterrupt:
        ending() 
        time.sleep(0.5)
        board.shutdown()
        print("Shutting down system.")

def led_display(seq, regNum):

    """
    Displays a binary sequence on a specified shift register for LED output.

    This function writes a sequence of binary values to one of three shift registers
    (1, 2, or 3), each connected to an LED array. It uses serial communication to 
    shift data into the register and latch the output.

    Parameters:
    seq (list of int): A list of 0s and 1s representing the binary sequence to display.
    regNum (int): The register number (1, 2, or 3) that selects which LED shift register to control.

    Returns:
    None
    """

    if regNum == 1:
        for index in range(len(seq)):
            board.digital_write(serialPin1, seq[index]) 
            board.digital_write(srclkPin1, 1)
            board.digital_write(srclkPin1, 0)
        board.digital_write(rclkPin1, 1)
        board.digital_write(rclkPin1, 0)
    elif regNum == 2:
        for index in range(len(seq)):
            board.digital_write(serialPin2, seq[index]) 
            board.digital_write(srclkPin2, 1)
            board.digital_write(srclkPin2, 0)
        board.digital_write(rclkPin2, 1)
        board.digital_write(rclkPin2, 0)
    elif regNum == 3:
        for index in range(len(seq)):
            board.digital_write(serialPin3, seq[index]) 
            board.digital_write(srclkPin3, 1)
            board.digital_write(srclkPin3, 0)
        board.digital_write(rclkPin3, 1)
        board.digital_write(rclkPin3, 0)
    else:
        print("Invalid shift register number.")

def get_distance(trigPin, trigPinNum, samples=7):

    """
    Measures distance using an ultrasonic sensor with multiple samples for accuracy.

    This function reads distance data from one of three ultrasonic sensors based on
    the trigger pin number. It performs multiple readings and returns the average
    of valid measurements to reduce noise and increase reliability.

    Parameters:
    trigPin (int): The trigger pin for the ultrasonic sensor (not directly used in logic).
    trigPinNum (int): Identifier for selecting the sensor (1, 2, or 3).
    samples (int): Number of samples to take for averaging (default is 7).

    Returns:
    float or None: The averaged distance reading if at least 3 valid samples are available;
                   otherwise, returns None.
    """

    readings = []
    for item in range(samples):
        if trigPinNum == 1:
            distance = board.sonar_read(us1Trig)[0]
        elif trigPinNum == 2:
            distance = board.sonar_read(us2Trig)[0]
        elif trigPinNum == 3:
            distance = board.sonar_read(us3Trig)[0]
        else:
            distance = None
        if distance is not None and distance > 0:
            readings.append(distance)
    if len(readings) >= 3:
        return sum(readings)/len(readings)
    else:
        return None

def starting():

    """
    Initializes the system state and resets all subsystem indicators.

    This function performs the following tasks:
    - Turns off the tone for subsystem 1.
    - Displays initial LED patterns for all three shift registers, representing subsystems 1 to 4.
    - Resets all iteration counters and first-time flags for the subsystems.

    Parameters:
    None

    Returns:
    None
    """
        
    global iteration1, iteration2, iteration3, iteration4
    #sub 1
    board.play_tone_off(pb2)
    led_display([0,1,0,0,1,0,0,0],1)
    #sub 2 & 4
    led_display([0,1,0,0,1,1,0,0],2)
    #sub 3
    led_display([0,0,1,0,0,0,0,0],3)
    iteration1 = 0
    iteration2 = 0
    iteration3 = 0
    iteration4 = 0
    firstTimeSub1 = False
    firstTimeSub2 = False
    firstTimeSub3 = False
    firstTimeSub4 = False

def ending():

    """
    Shuts down all subsystems by clearing LED displays and stopping sound.

    This function performs the following shutdown actions:
    - Turns off any sound playing through the speaker 
    - Clears all LED outputs on shift registers 1, 2, and 3 by sending zeroed patterns.

    Parameters:
    None

    Returns:
    None
    """

    board.play_tone_off(pb2)
    led_display([0,0,0,0,0,0,0,0],1) 
    led_display([0,0,0,0,0,0,0,0],2) 
    led_display([0,0,0,0,0,0,0,0],3) 

def read_push_button():

    """
    Reads the state of a digital push button.

    This function checks the digital state of the push button connected to pin PB1.
    If the button is pressed (HIGH signal), it returns True. Otherwise, it returns False.

    Parameters:
    None

    Returns:
    bool: True if the button is pressed, False otherwise.
    """

    stateA = board.digital_read(pb1)[0]
    
    if stateA == 1:
        return True
    else:
        return False

def read_all_us():

    """
    Reads distance measurements from all ultrasonic sensors and updates global variables.

    This function calls `get_distance()` for each of the three ultrasonic sensors
    (US1, US2, and US3), and stores the measured distances in corresponding global variables.

    Parameters:
    None

    Returns:
    None
    """

    global objectDistanceUS1, objectDistanceUS2, objectDistanceUS3
    
    objectDistanceUS1 = get_distance(us1Trig, 1)
    objectDistanceUS2 = get_distance(us2Trig, 2)
    objectDistanceUS3 = get_distance(us3Trig, 3)

def refresh_state():

    """
    Updates the overheight vehicle detection state for all ultrasonic sensors.

    This function performs the following:
    - Reads the latest distances from US1, US2, and US3.
    - Updates flags indicating whether an overheight vehicle is currently detected at each sensor.
    - Determines whether a vehicle has exited the detection zone since the last update.
    - Sets additional system flags such as `override4I3Active` and `readyToReset4I3` for controlling subsystem logic.

    Parameters:
    None

    Returns:
    None
    """

    global objectDistanceUS1, objectDistanceUS2, objectDistanceUS3
    global overheightVechicleAtUS1, overheightVechicleAtUS2, overheightVechicleAtUS3
    global overheightVechicleExitedUS1, overheightVechicleExitedUS2, overheightVechicleExitedUS3, override4I3Active, readyToReset4I3

    objectDistanceUS1 = get_distance(us1Trig, 1)
    objectDistanceUS2 = get_distance(us2Trig, 2)
    objectDistanceUS3 = get_distance(us3Trig, 3)

    # Track current status
    wasUS1 = overheightVechicleAtUS1
    wasUS2 = overheightVechicleAtUS2
    wasUS3 = overheightVechicleAtUS3

    # Update "At" flags
    overheightVechicleAtUS1 = objectDistanceUS1 is not None and objectDistanceUS1 < detectionThreshold
    overheightVechicleAtUS2 = objectDistanceUS2 is not None and objectDistanceUS2 < detectionThreshold
    overheightVechicleAtUS3 = objectDistanceUS3 is not None and objectDistanceUS3 < detectionThreshold

    # If it was there, but now it isn't → mark as exited
    if wasUS1 and not overheightVechicleAtUS1:
        overheightVechicleExitedUS1 = True
        overheightVechicleAtUS1 = False
    else:
        overheightVechicleExitedUS1 = False

    if wasUS2 and not overheightVechicleAtUS2:
        overheightVechicleExitedUS2 = True
        overheightVechicleAtUS2 = False
    else:
        overheightVechicleExitedUS2 = False

    if wasUS3 and not overheightVechicleAtUS3:
        overheightVechicleExitedUS3 = True
        overheightVechicleAtUS3 = False
        readyToReset4I3 = True
    else:
        overheightVechicleExitedUS3 = False

    # If US2 detects anything at all → activate override
    if overheightVechicleAtUS2:
        override4I3Active = True

def sub3i2(startTime):

    """
    Controls the LED states and timing logic for subsystem 3 integrate subsystem 2 based on elapsed time and button press.

    This function:
    - Tracks elapsed time since `startTime` to manage LED color transitions on shift register 2.
    - Changes LEDs according to a time schedule: yellow before 2 seconds, red between 2 and 7 seconds, then green after 7 seconds.
    - Monitors a push button press to immediately switch the LED pattern to green and turn off red.
    - Manages the running state flag `running3I2` and updates the subsystem start time.

    Parameters:
    startTime (float): The timestamp when subsystem 3 integrate subsystem 2 started, used for elapsed time calculation.

    Returns:
    None
    """

    global running3I2, startTime3I2
    timeDiff = time.time() - startTime
    print("check_pb")

    if running3I2 == False: 
        startTime3I2 = startTime
        running3I2 = True
        stateShiftRegister2[redPL1] = 1

    if read_push_button() == True:
        print("detetcted")
        stateShiftRegister2[greenPL1] = 1
        stateShiftRegister2[redPL1] = 0
        led_display(stateShiftRegister2,2)
    if timeDiff < 2:
        stateShiftRegister2[yellowTL4] = 1
        stateShiftRegister2[greenTL4] = 0
        led_display(stateShiftRegister2,2)
    elif timeDiff < 7:
        stateShiftRegister2[yellowTL4] = 0
        stateShiftRegister2[redTL4] = 1
        stateShiftRegister2[redPL1] = 0
        led_display(stateShiftRegister2,2)
    else:
        print("done3i2")
        stateShiftRegister2[greenTL4] = 1
        stateShiftRegister2[redTL4] = 0
        stateShiftRegister2[greenPL1] = 0
        stateShiftRegister2[redPL1] = 1
        led_display(stateShiftRegister2,2)
        running3I2 = False

def subsystem1(startTime):

    """
    Controls the behavior of subsystem 1, handling LED patterns and buzzer tones for overheight vehicle detection.

    The function manages a sequence of LED displays and buzzer sounds based on the elapsed time since `startTime`.
    It includes an initial measurement of vehicle height, warning signals with increasing urgency,
    and a blinking LED pattern with corresponding buzzer frequencies.

    The function respects an override flag to pause its execution and resets state variables when the subsystem ends.

    Parameters:
    startTime (float): Timestamp marking when subsystem 1 started, used to calculate elapsed time.

    Returns:
    None
    """

    global runningSub1, iteration1, override4I3Active, firstTimeSub1

    if override4I3Active:
        return
    
    if firstTimeSub1 == False:
        firstTimeSub1 = True
        objectDistanceUS1 = get_distance(us1Trig, 1) 
        vechicleHeightMetre = (30 - objectDistanceUS1)*20 / 100 # 20:1 (real : prototype) ratio
        print("Overheight vechicle still detected. Frequency increase to 2500 Hz")
        print(f"Height of OVERHEIGHT VECHICLE in metre: {vechicleHeightMetre} m")

    print("Start S1")
    timeDiff = time.time() - startTime

    if timeDiff < 1:
        board.play_tone_off(pb2)
        seq1 = [0,1,0,0,0,1,0,1]
        led_display(seq1, 1)

    elif 1 <= timeDiff < 2:
        seq2 = [0,0,1,0,0,0,1,1]
        led_display(seq2,1)
        board.play_tone(pb2, 3000, 1000) # passize buzzer sounds at 3000Hz

    elif 2 <= timeDiff < 32: 
        board.play_tone(pb2, 3000, 30000)
        seqFlashOn = [1,0,0,1,0,0,1,1]
        seqFlashOff = [0,0,0,1,0,0,1,1]
        iteration1 +=1
        if iteration1 % 2 == 1:
            led_display(seqFlashOn,1)
        else:
            led_display(seqFlashOff,1)

    elif timeDiff >= 32:
        objectDistanceUS1 = get_distance(us1Trig, 1) 

        if objectDistanceUS1 < detectionThreshold:
            board.play_tone(pb2, 5000, 120000000000000000) # passive buzzer sounds at 5000Hz
        seqFlashOn = [1,0,0,1,0,0,1,1]
        seqFlashOff = [0,0,0,1,0,0,1,1]
        iteration1 +=1
        if iteration1 % 2 == 1:
            led_display(seqFlashOn,1)
        else:
            led_display(seqFlashOff,1)
        if us3CameAndLeft == True and objectDistanceUS1 < detectionThreshold:
            print("S1 done")
            board.play_tone_off(pb2)
            led_display([0,1,0,0,1,0,0,0],1)
            runningSub1 = False 
            iteration1 = 0
            firstTimeSub1 = False

def starting_sub1():

    """
    Reset subsystem 1 indicators by turning off the buzzer and setting the LED pattern.

    This function stops any buzzer sound on PA1 and updates the LEDs of shift register 1
    to the initial pattern [0,1,0,0,1,0,0,0], indicating the subsystem reset state.

    Parameters:
    None

    Returns:
    None
    """
 
    board.play_tone_off(pb2)
    led_display([0,1,0,0,1,0,0,0],1)

def subsystem2(startTime):

    """
    Manage the behavior and LED signaling of subsystem 2 based on elapsed time since start.

    This function controls the timing and LED states for subsystem 2:
    - On the first call, it registers the button press.
    - Between 0-2 seconds: subsystem runs without LED changes.
    - Between 2-4 seconds: sets yellow traffic light on TL4 and green on TL3; red pedestrian light on PL1.
    - Between 4-7 seconds: switches TL4 light from yellow to red; pedestrian light PL1 changes to green.
    - Between 7-9 seconds: blinks red pedestrian light PL1.
    - After 9 seconds: resets the lights to red on PL1, green on TL4, turns off red on TL4, marks subsystem as stopped, and resets counters.

    Parameters:
    startTime (float): The start time timestamp of the subsystem operation.

    Returns:
    None
    """

    global iteration, timePrevious2, runningSub2, lastState2, firstTimeSub2
    print("Start S2")
    timeDiff = time.time() - startTime

    if firstTimeSub2 == False:
        firstTimeSub2 = True
        print("Button pressed")

    if timeDiff < 2:
        runningSub2 = True
        
    elif 2 < timeDiff < 4:
        runningSub2 = True
        stateShiftRegister2[greenTL4] = 0 
        stateShiftRegister2[yellowTL4] = 1
        stateShiftRegister2[greenTL3] = 1
        stateShiftRegister2[redPL1] = 1
        led_display(stateShiftRegister2,2)
    elif 4 < timeDiff < 7:
        runningSub2 = True
        stateShiftRegister2[yellowTL4] = 0 
        stateShiftRegister2[redTL4] = 1
        stateShiftRegister2[redPL1] = 0
        stateShiftRegister2[greenPL1] = 1
        led_display(stateShiftRegister2,2)

    if 7 < timeDiff < 9:
        stateShiftRegister2[greenPL1] = 0
        iteration+=1
        if iteration%2 == 1:
            stateShiftRegister2[redPL1] = 1
        else:
            stateShiftRegister2[redPL1] = 0

        led_display(stateShiftRegister2,2)
        
        print(f"This is iteration {iteration}")
    if timeDiff > 9:
        print("enter 9")
        stateShiftRegister2[redPL1] = 1
        stateShiftRegister2[redTL4] = 0
        stateShiftRegister2[greenTL4] = 1
        led_display(stateShiftRegister2,2)
        stateA = board.digital_read(pb1)[0]
        lastState2 = stateA
        timePrevious2 = time.time()
        runningSub2 = False
        print("S2 done")
        iteration  = 0
        firstTimeSub2 = False
        
def subsystem3(startTime):

    """
    Control the behavior and LED signaling of subsystem 3 based on elapsed time.

    This function manages subsystem 3 in phases according to time since startTime:
    - For the first 2 seconds:
      * Resets the 'us3CameAndLeft' flag.
      * Calls sub3i2 to handle button press logic.
      * Displays a specific LED pattern on shift register 3.
    - Between 2 and 7 seconds:
      * Displays a different LED pattern.
      * Measures distance with ultrasonic sensor 3.
      * If distance exceeds detection threshold, sets 'us3CameAndLeft' to True.
    - After 7 seconds:
      * Measures distance again.
      * If an object is detected within threshold, flashes green LEDs alternately.
      * If no object detected, displays a steady LED pattern,
        marks subsystem 3 as done, resets iteration counter and sets 'us3CameAndLeft' to True.

    Parameters:
    startTime (float): The timestamp marking the start of subsystem 3 operation.

    Returns:
    None
    """

    global runningSub3, iteration3, override4I3Active, us3CameAndLeft 
    
    print("Start S3")
    timeDiff = time.time() - startTime 

    if timeDiff < 2:
        us3CameAndLeft = False
        sub3i2(startTime)
        seq1 = [0,1,0,0,0,0,0,0]
        led_display(seq1, 3)

    elif 2 < timeDiff < 7:
        seq2 = [1,0,0,0,0,0,0,0]
        led_display(seq2, 3)

        objectDistanceUS3 = get_distance(us3Trig, 3)

        if objectDistanceUS3 > detectionThreshold:
            us3CameAndLeft = True

    elif timeDiff > 7:
        objectDistanceUS3 = get_distance(us3Trig, 3)

        if objectDistanceUS3 < detectionThreshold:
            flashGreenSeq1 = [1,0,0,0,0,0,0,0] 
            flashGreenSeq2 = [0,0,0,0,0,0,0,0]

            iteration3 +=1
            if iteration3 % 2 == 1:
                led_display(flashGreenSeq1,3)
            else:
                led_display(flashGreenSeq2,3)
        else:
            led_display([0,0,1,0,0,0,0,0],3)
            print("S3 done")
            runningSub3 = False
            iteration3 = 0
            us3CameAndLeft = True

def subsystem4(startTime):

    """
    Manage subsystem 4 behavior and LED signaling based on ultrasonic sensor 2 detection.

    This function performs the following steps:
    - Stops runningSub2 and resets its iteration and timer.
    - Prints status messages indicating subsystem start and US2 detection.
    - Sets specific LEDs on shift register 2 to represent the current state.
    - Reads the distance from ultrasonic sensor 2 (US2).
    - If an object is detected within the threshold distance:
      * Updates LED states on shift register 2 to indicate detection.
      * Blinks a warning LED (redWL2) on and off alternately using iteration count.
      * Continuously refreshes the object distance reading.

    Parameters:
    startTime (float): The timestamp marking the start of subsystem 4 operation.

    Returns:
    None
    """
        
    global runningSub2, timePrevious2, iteration4, iteration, runningSub4, override4I3Active 

    print("Start s4")

    runningSub2 = False
    iteration = 0
    timePrevious2 = time.time()

    timeDiff = time.time() - startTime

    print("US2 detected") 
    stateShiftRegister2[redTL3] = 1
    stateShiftRegister2[greenTL3] = 0
    stateShiftRegister2[redPL1] = 1
    stateShiftRegister2[greenPL1] = 0
    led_display(stateShiftRegister2,2)

    objectdistance = get_distance(us2Trig, 2)
    if not (objectdistance is None or objectdistance > detectionThreshold):
        print('US2 detected')
        stateShiftRegister2[redTL4] = 1
        stateShiftRegister2[yellowTL4] = 0
        stateShiftRegister2[greenTL4] = 0

        iteration4 +=1
        if iteration4 % 2 == 1:
            stateShiftRegister2[redWL2] = 1
            led_display(stateShiftRegister2,2) 
        else:
            stateShiftRegister2[redWL2] = 0
            led_display(stateShiftRegister2, 2)  
        objectdistance = get_distance(us2Trig, 2)
        
def check_reset_4I3():

    """
    Check conditions and reset the subsystem 4 integrate subsystem 3 system to normal state if criteria are met.

    This function monitors the ultrasonic sensors and internal flags to determine if the
    system should reset from an override or alert state back to normal operation.
    The reset happens only if:
    - readyToReset4I3 is True,
    - no overheight vehicles are detected by any ultrasonic sensors (distances above threshold or None),
    - and the flag us3CameAndLeft indicates the vehicle has left the last sensor.

    Upon reset:
    - Subsystem 1 LEDs are reset to their normal state.
    - Shift register 2 LEDs controlling various traffic lights are set to normal.
    - Subsystem 3's traffic light is set to red.
    - Override and running flags are cleared.

    Parameters:
    None

    Returns:
    None
    """

    global override4I3Active, runningSub1, runningSub3, runningSub4
    global stateShiftRegister2, readyToReset4I3

    objectDistanceUS1 = get_distance(us1Trig, 1)
    objectDistanceUS2 = get_distance(us2Trig, 2)
    objectDistanceUS3 = get_distance(us3Trig, 3)
    
    print('=======================================')

    if (
        readyToReset4I3 and
        (objectDistanceUS1 is None or objectDistanceUS1 > detectionThreshold) and
        (objectDistanceUS2 is None or objectDistanceUS2 > detectionThreshold) and 
        us3CameAndLeft
    ):
        print("4.I3: Resetting system to normal state")

        led_display([0,1,0,0,1,0,0,0], 1)

        stateShiftRegister2[redTL3] = 0  
        stateShiftRegister2[greenTL3] = 1  
        stateShiftRegister2[redWL2] = 0
        stateShiftRegister2[greenTL4] = 1  
        stateShiftRegister2[yellowTL4] = 0 
        stateShiftRegister2[redTL4] = 0 
        led_display(stateShiftRegister2, 2)


        led_display([1,0,0,0,0,0,0,0], 3)  

        override4I3Active = False
        runningSub1 = False
        runningSub3 = False
        runningSub4 = False 
        readyToReset4I3 = False

if __name__ == '__main__':
    main(startTime)
