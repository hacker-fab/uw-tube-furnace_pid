import serial
import time
import csv
import matplotlib.pyplot as plt
import numpy as np
import msvcrt

class DeltaDTB: #Hold SET Button, click cycle until seeing Co5H, turn it on to enable Communication Write Enable parameter (Allow modifying setting through python code)
    """
    # Modbus ASCII Format
    # Request Frame (Master to Slave): Colon :(1), Slave ID (2), Function (2), Address (4), Quantity (4), EB- LRC (redundancy check), \r\n- carriage return
    # Response Frame (Slave to Master):: Colon :(1), Slave ID (2), Function (2), Byte Count (2), Data (4), EB- LRC (redundancy check), \r\n- carriage return
    """
    # --- Modbus ASCII Frame Offsets ---
    DATA_START          = 7  # The index where the first register's hex data begins
    HEX_PER_REG         = 4  # Each 16-bit register is represented by 4 hex chars

    # --- Address Map (Class Constants) ---
    # Basic Control Addresses
    UNLOCK              = 0x102C  # Setting lock status Need to set
    HEAT_COOL_SEL       = 0x1006  # Heating/Cooling selection 0: Heating, 1: Cooling, 2: Heating/Cooling, 3: Cooling/Heating
    CTRL_METHOD         = 0x1005  # Control method (PID/Program/Manual) 0: PID, 1: ON/OFF, 2: manual tuning, 3: PID program control
    CYCLE_TIME          = 0x1007  # Relay Cycle: Control cycle of Output 1
    RUN_STOP            = 0x103C  # Run/Stop command  0:Stop, 1:Run, 2:End, 3:Hold

    # Reading
    PV                  = 0x1000  # Process Value Measuring unit is 0.1, updated one time in 0.4 second
    PID_P               = 0x1009  # Proportional Band
    PID_I               = 0x100A  # Integral Time
    PID_D               = 0x100B  # Derivative Time
    OUT1_VOL            = 0x1012  # Power %: On/Off Duration Ratio in Relay Cycle

    # Safety & Limit Addresses
    SENSOR_TYPE         = 0x1004  # CnPt Thermocouple: K, J, T, E, N, R, S, B, L, U, TXK (3-wire Platinum RTD: Pt100, JPt100; Analog input: 0 ~ 5V, 0 ~ 10V, 0 ~ 20 m A, 4 ~ 20 m A, 0 ~ 50mV)
    LIMIT_LOW           = 0x1003  # t-L: Lower-limit of temp range
    LIMIT_HIGH          = 0x1002  # t-H: Upper-limit of temp range
    SV                  = 0x1001  # SV Set Value
    """
    SENSOR_TYPE Value (Temperature Range):
    0: Thermocouple K -200 ~ 1,300°C
    1: Thermocouple J -100 ~ 1,200°C
    2: Thermocouple T -200 ~ 400°C
    3: Thermocouple E 0 ~ 600°C
    4: Thermocouple N -200 ~ 1,300°C
    5: Thermocouple R 0 ~ 1,700°C
    6: Thermocouple S 0 ~ 1,700°C
    7: Thermocouple B 100 ~ 1,800°C
    8: Thermocouple L -200 ~ 850°C
    9: Thermocouple U -200 ~ 500°C
    10: Thermocouple TXK -200 ~ 800°C
    11: Platinum Resistance (JPt100) -20 ~ 400°C
    12: Platinum Resistance (Pt100) -200 ~ 600°C
    13: 0V ~ 5V Analog Input -999 ~ 9,999
    14: 0V ~ 10V Analog Input -999 ~ 9,999
    15: 0 ~ 20mA Analog Input -999 ~ 9,999
    16: 4 ~ 20mA Analog Input -999 ~ 9,999
    17: 0 ~ 50mV Analog Input -999 ~ 9,999
    """

    # Program / Pattern Base Addresses
    START_PATTERN       = 0x1030  # S-PA Start pattern number
    AUTO_TUNE           = 0x103B  # Auto Tune for PID Parameter setting OFF: 0 (default), ON : 1; Control method: PID mode!!!
    TEMP_BASE           = 0x2000 # Pattern 0, Step 0 Setpoint Temperature
    TIME_BASE           = 0x2080 # Pattern 0, Step 0 Duration Time
    ACTUAL_STEP_BASE    = 0x1040 # Actual step number (0 ~ 7 = N, indicate that this pattern is executed from step 0 to step N)
    LINK_PTN_BASE       = 0x1060 # Pattern linking (0 ~ 8, 8 indicates the program end. 0~7 indicates the next execution pattern number after executing the current pattern
    PTN_CYCLE_BASE      = 0x1050 # Repeating number of the 0 ~ 7 pattern (0 ~ 99 indicate that this pattern has been executed for 1 ~ 100 times)

    # Execution Status Addresses
    TIME_REMAIN_SEC = 0x103D  # Time remaining of present step (Seconds)
    TIME_REMAIN_MIN = 0x103E  # Time remaining of present step (Minutes)
    CURR_STEP       = 0x1034  # Present executing step number
    CURR_PATTERN    = 0x1035  # Present executing pattern number
    CURR_CYCLE      = 0x1036  # Present pattern cycle current count

    def __init__(self, port, baud, slave_id=1):
        self.ser = serial.Serial(port, baud, bytesize=7, parity='E', stopbits=1, timeout=1)
        self.slave_id = slave_id
    
    def parse_response(self, resp, reg_index=0):
        """
        Extracts the hex string for a specific register index from the response.
        reg_index=0 is the first register, 1 is the second, etc.
        """
        start = self.DATA_START + (reg_index * self.HEX_PER_REG)
        end = start + self.HEX_PER_REG
        return resp[start:end].decode().upper()
    
    def _calculate_lrc(self, hex_string):
        """Calculates the LRC checksum for Modbus ASCII."""
        byte_sum = sum(int(hex_string[i:i+2], 16) for i in range(0, len(hex_string), 2))
        lrc = (0x100 - (byte_sum & 0xFF)) & 0xFF
        return f"{lrc:02X}"
    
    def write_register(self, address, value):
        """Sends Function 06 (Write) and verifies the Echo for success/failure."""
        addr_hex = f"{address:04X}"
        val_hex = f"{value:04X}"
        payload = f"{self.slave_id:02X}06{addr_hex}{val_hex}"
        cmd = f":{payload}{self._calculate_lrc(payload)}\r\n".encode('ascii')
        self.ser.write(cmd)
        time.sleep(0.2) # Wait for controller
        resp = self.ser.read(100)
    
        # Fallout Debug Logic
        if not resp:
            print(f"ERROR: No response on write to {hex(address)}")
        elif resp.strip() == cmd.strip():
            print(f"SUCCESS: {hex(address)} set to {value}")
        elif b'86' in resp[3:5]: # Exception check (06 + 80 = 86)
            print(f"REJECTED: {hex(address)} Exception Code: {resp[5:7].decode()}")
        else:
            print(f"UNEXPECTED: Response was {resp}")
        return resp
    
    def read_registers(self, start_address, count=1):
        """Sends Function 03 (Read) and handles data corruption/serial errors."""
        try:
            addr_hex = f"{start_address:04X}"
            count_hex = f"{count:04X}"
            payload = f"{self.slave_id:02X}03{addr_hex}{count_hex}"
            cmd = f":{payload}{self._calculate_lrc(payload)}\r\n".encode('ascii')
            self.ser.write(cmd)
            time.sleep(0.1)
            resp = self.ser.read(100)
        
            if not resp:
                print(f"ERROR: Read timeout at {hex(start_address)}")
                return None
            
            # Basic Modbus ASCII frame validation (starts with : and ends with \r\n)
            if not resp.startswith(b':') or not resp.endswith(b'\r\n'):
                print(f"CORRUPTED: Invalid frame from {hex(start_address)}")
                return None
                
            return resp
        
        except serial.SerialException as e:
            print(f"SERIAL ERROR: {e}. Reconnecting...")
            time.sleep(2)
            return None
    
    # --- Read Data ---
    def get_pv(self, read_qty = 1):
        """Reads current temperature (Process Value)"""
        resp = self.read_registers(self.PV, read_qty)
        if resp and len(resp) >= 11: # (7 header + (1*4) data + 2 LRC + 2 \r\n)
            try:
                # Extract the hex data
                hex_val = self.parse_response(resp)
                val_dec = int(hex_val, 16)
                # Hardware Error Check logic
                error_codes = {
                    "8002": "Initializing (8002H)", # Initial process (Temperature value is not got yet)
                    "8003": "Sensor Disconnected (8003H)", # Temperature sensor is not connected
                    "8004": "Sensor Input Error (8004H)", # Temperature sensor input error
                    "8006": "ADC Input Error (8006H)", # Cannot get temperature value, ADC input error
                    "8007": "Memory Error (8007H)" # 8007H : Memory read/write error
                }
                if hex_val in error_codes:
                    print(f"FURNACE ERROR: {error_codes[hex_val]}")
                    return None       
                # Return the temperature if no error
                return val_dec / 10.0
                
            except (ValueError, UnicodeDecodeError, IndexError):
                print("DEBUG: Process Value Hex Parsing Failed")
        return None

    def get_pid_values(self, read_qty = 3):
        """Reads P, I, and D values (Addresses 1009H - 100BH) in one request using dynamic parsing."""
        resp = self.read_registers(self.PID_P, read_qty)
        if resp:
            try:
                # Indices for multi-register read:
                # Reg1 (P): 7-11 | Reg2 (I): 11-15 | Reg3 (D): 15-19
                p = int(self.parse_response(resp, 0), 16) / 10.0
                i = int(self.parse_response(resp, 1), 16)
                d = int(self.parse_response(resp, 2), 16)
                return p, i, d
            except:
                print("DEBUG: PID Parsing Failed")
        return (0.0, 0, 0)

    def get_power_percent(self, read_qty = 1):
        """Reads Output Power % (Address 1012H)"""
        resp = self.read_registers(self.OUT1_VOL, read_qty)
        if resp:
            try:
                # Unit is 0.1%, so 1000 = 100.0%
                return int(self.parse_response(resp), 16) / 10.0
            except:
                print("DEBUG: Power Parsing Failed")
        return 0.0

    @staticmethod
    def calculate_slew_rate(times, temps, window=3):
        """
        Calculates the rate of change (°/min) using linear regression 
        over the last 'window' data points.
        """
        # Return 0 during start
        if len(temps) < 2: return 0.0
        # Extract the most recent N points
        if len(temps) >= window:
            # Fit a 1st-degree polynomial (line): y = mx + c
            # slope (m) is the first element of the returned array
            x, y = np.array(times[-window:]), np.array(temps[-window:])
            slope, _ = np.polyfit(x, y, 1)
            return slope * 60 # per minute
        # Fallback to simple difference if not enough points for regression
        return (temps[-1] - temps[-2]) / (times[-1] - times[-2]) * 60
    
    # --- Initialize Setup ---
    def initialize_hardware(self, relay_period, low_limit = 0, high_limit = 1150.0): # 0°C - 1150°C, Relay Cycle Time (s) 0 ~ 99, 0:0.5 sec 
        """Run once to unlock and set safe physical boundaries."""
        print("--- Initializing Hardware ---")
        self.write_register(self.UNLOCK, 0)
        self.write_register(self.SENSOR_TYPE, 0) # K-type Thermocouple value is 0
        self.write_register(self.SV, 0) # Clear the SV to 0 to prevent limit conflicts
        self.write_register(self.LIMIT_HIGH, int(high_limit * 10)) # Set Upper limit first to avoid conflict of lower limit, expand the allowable range (Exception Code: 03 Rejected maybe because the existing upper limit is already 1150.0°C.)
        self.write_register(self.LIMIT_LOW, int(low_limit * 10))
        self.write_register(self.HEAT_COOL_SEL, 0) # Force Heating mode
        self.write_register(furnace.CTRL_METHOD, 3) # Set to 3: PID program control
        self.write_register(self.CYCLE_TIME, relay_period) # relay_period: The time window for the duty cycle, longer period extends mechanical relay life during pulsing.

    # --- Pattern, Step, Flow Setup ---
    def set_pattern_step(self, pattern, step, temp, minutes):
        """
        Sets a specific temperature and time for any of the 64 steps.
        pattern: 0-7, step: 0-7
        """
        # Logic: There are 8 steps per pattern.
        # Pattern 0 starts at +0, Pattern 1 starts at +8, etc.
        offset = (pattern * 8) + step
        
        # Calculate final hex addresses
        temp_addr = self.TEMP_BASE + offset
        time_addr = self.TIME_BASE + offset
        
        # Write values
        self.write_register(temp_addr, int(temp * 10))
        self.write_register(time_addr, int(minutes))
    
    def config_pattern_logic(self, pattern, actual_steps, link_to=8, repeat=0):
        """
        Configures how a pattern behaves.
        actual_steps: 0-7 (How many steps to execute)
        link_to: 0-7 (Next pattern) or 8 (Stop)
        repeat = 0 means run once. repeat = 1 means run twice.
        """
        # Both registers are also contiguous for patterns 0-7
        self.write_register(self.ACTUAL_STEP_BASE + pattern, actual_steps)
        self.write_register(self.LINK_PTN_BASE + pattern, link_to)
        self.write_register(self.PTN_CYCLE_BASE + pattern, repeat)
    
    def load_profile(self, pattern, profile, next_ptn=8, repeat=0):
        """
        Automates pattern programming.
        profile: A list of tuples [(temp, mins), (temp, mins), ...]
        Example: [(100, 10), (100, 30)] -> Ramp to 100 in 10m, Soak at 100 for 30m.
        """
        num_steps = len(profile)
        if num_steps > 8:
            raise ValueError(f"Pattern {pattern} exceeded 8 steps.")
        
        # Program each step in the "steps" list from the input Dictionary sub Dictionary (PID_Program/config/"steps")
        for i, (temp, mins) in enumerate(profile):
            self.set_pattern_step(pattern, i, temp, mins)

        # Program the pattern flow
        actual_steps = num_steps - 1 # 0 ~ 7 Step Number
        self.config_pattern_logic(pattern, actual_steps, next_ptn, repeat)
        print(f"Pattern {pattern} Loaded: {num_steps} steps, Next: {next_ptn}, Repeat: {repeat}")
    
    def load_flow(self, flow_dict):
        """
        Programs patterns and their flow logic (links and repeats).
        """
        self.stop()
        print("--- Programming Modular Flow ---")
        
        for p_idx, config in flow_dict.items():
            self.load_profile(
                pattern = p_idx, 
                profile = config["steps"], 
                next_ptn = config.get("next", 8), 
                repeat = config.get("repeat", 0)
            )

        print("--- Flow Loaded Successfully ---")

    def get_execution_status(self):
        """
        Reads the hardware status to see exactly where the furnace is.
        Returns: Pattern, Step, Time_Remaining (Seconds)
        """
        # Read Step and Pattern (Contiguous)
        # 0x1034 (Step), 0x1035 (Pattern)
        pos_data = self.read_registers(self.CURR_STEP, 2)
        
        # 2. Read Time Remaining (Separate block at 103DH)
        time_data = self.read_registers(self.TIME_REMAIN_SEC, 1)

        if pos_data and time_data:
            try:
                # Parsing based on your parse_response (assuming index refers to register position)
                step    = int(self.parse_response(pos_data, 0), 16)
                pattern = int(self.parse_response(pos_data, 1), 16)
                sec_rem = int(self.parse_response(time_data, 0), 16)
                
                # Format seconds into MM:SS
                minutes, seconds = divmod(sec_rem, 60)
                time_str = f"{minutes:02d}:{seconds:02d}"
                
                return pattern, step, time_str, sec_rem
            except (ValueError, TypeError, IndexError) as e:
                print(f"DEBUG: Status Parse Error: {e}")
        else:
            print(f"DEBUG: Status Read Failed/Parse Error: (Pos: {len(pos_data) if pos_data else 0}, Time: {len(time_data) if time_data else 0})")
        
        # Return safe defaults if anything fails to prevent script crash
        return 0, 0, "00:00", 0
        
    def live_load_profile(self, new_profile_dict):
        """Allows user to overwrite the current sequence without stopping the script."""
        print("\n[!] INTERRUPT: Loading new profile...")
        try:
            # Keep the furnace in HOLD here so the heater keeps running 
            # while we overwrite the memory.
            self.load_flow(new_profile_dict)

            # Force the controller to look at Pattern 0, Step 0 immediately
            # Resetting Start Pattern (1030H) and clearing current timer (1032H)
            self.write_register(self.START_PATTERN, 0)
            self.write_register(self.TIME_REMAIN_SEC, 0) 

            # Release the HOLD and Resume
            self.hold(active=False)
            print("--- New Profile Active and Jumped to P0 S0 ---")
        except Exception as e:
            print(f"FAILED to load profile: {e}")
            self.hold(active=False) # Resume so it doesn't stay stuck

    # --- PID Tuning ---
    def set_tuning(self, p, i, d):
        """Sets the P, I, and D parameters."""
        self.write_register(self.PID_P, int(p * 10))
        self.write_register(self.PID_I, int(i))
        self.write_register(self.PID_D, int(d))
    
    def start_auto_tune(self):
        """Triggers the Auto-Tune (AT) process (0813H)"""
        print("--- Starting Auto-Tune ---")
        # Write 1 to start the tuning process
        return self.write_register(self.AUTO_TUNE, 1)
    
    def monitor_at_progress(self):
        """Checks the AT bit (0x103B) and returns immediately."""
        try:
            at_status = self.read_register(self.AUTO_TUNE)
            return (at_status == 1)
        except Exception:
            # If the serial line is busy or glitches, return the last known state
            # to prevent the script from jumping out of AT mode prematurely.
            return getattr(self, '_last_at_state', False)
        
    # --- Execution Command ---
    def run(self):
        """Sets hardware to RUN mode (Value 1)."""
        print("Start Running...")
        return self.write_register(self.RUN_STOP, 1)
    
    def stop(self):
        """Hardware Hard Stop (Ends Program, Cuts Power)."""
        print("Stop Running...")
        return self.write_register(self.RUN_STOP, 0)

    def hold(self, active=True):
        """
        Toggles the 'Hold' state. 
        3: Hold (Pause timer), 1: Run (Resume timer).
        """
        val = 3 if active else 1
        state_text = "PAUSING (Hold)" if active else "RESUMING"
        print(f"{state_text} Program...")
        return self.write_register(self.RUN_STOP, val)
    
    def skip_step(self):
        """Forces the current step to end by setting remaining time to 0."""
        # Get current status before skipping
        status = self.get_execution_status() # (Ptn, Step, TimeStr, Sec)
        
        if status:
            ptn, step, time_left = status[0], status[1], status[2]
            print(f"\n>>> SKIPPING: Pattern {ptn}, Step {step} (Time Remaining: {time_left})")
        else:
            print("\n>>> SKIPPING: Could not retrieve current step info.")
        
        # SAFETY: Ensure we are in RUN mode (1) so the timer register is "unlocked"
        # If the user opened the manual menu, the furnace is currently in HOLD (3)
        self.write_register(self.RUN_STOP, 1) 
        time.sleep(0.2) # Small delay for hardware state transition

        return self.write_register(self.TIME_REMAIN_SEC, 0)

# Auto Tune Function
def prompt_auto_tune(furnace):
    while True:
        val = input("\nTarget Temp for Tuning (°C) or 'q' to cancel: ").strip().lower()
        
        if val == 'q':
            print("Aborting Auto-Tune. Resuming program...")
            return # Exit and do nothing

        try:
            target = float(val)
            current_temp = furnace.get_pv()
            
            # --- Safety Check: 50C Window ---
            if abs(target - current_temp) > 50:
                print(f"[!] SAFETY ERROR: Target ({target}°C) is too far from PV ({current_temp:.1f}°C).")
                print("AT requires a target within 50°C of current temp for accuracy.")
                continue # Re-ask the prompt
            
            # --- Execution ---
            # Ensure the controller is in PID mode (1005H = 0) in order for Auto Tune Process to run
            furnace.write_register(furnace.CTRL_METHOD, 0)
            time.sleep(0.5)
            # Set Manual Setpoint (1001H)
            furnace.write_register(furnace.SV, int(target * 10))
            time.sleep(0.5)
            # Start Auto Tune Process and Wait until it done
            furnace.start_auto_tune()
            break # Success, exit the while loop

        except ValueError:
            print(f"[!] '{val}' is not a valid number. Please try again.")

    # --- Resume to PID Program Control ---
    print("Resuming PID Program Control...")

# User Interference Function
def prompt_PID_Program_edit(furnace):
    print("\n--- NEW PID PROFILE BUILDER (override existing PID Program, start at Pattern 0) ---")
    print(" (Type 'q' at any time to cancel and resume existing PID Program)")
    New_PID_Program = {}
    # --- 1. Get Pattern Number ---
    try:
        while True: # Loop for each pattern
            # --- 1. Pattern Number ---
            while True:
                try:
                    val = input("Pattern Number (0-7): ").strip().lower()
                    if val == 'q': raise InterruptedError
                    p_idx = int(val)
                    if not 0 <= p_idx <= 7: raise ValueError("Pattern must be 0-7")
                    break # Success, move to next prompt
                except ValueError as e:
                    print(f"Invalid input: {e}")

            # --- 2. Steps ---
            while True:
                try:
                    steps_str = input("Steps(Maximum 8): Temperature(°C),Duration(min); Temperature(°C),Duration(min) (e.g. 100,10; 200,20): ").strip().lower()
                    if steps_str == 'q': raise InterruptedError
                    
                    # Safe Parsing
                    steps = []
                    for s in steps_str.strip(';').split(';'): # .strip(';') handles accidental trailing semicolons like "100,10;"
                        parts = s.split(',')
                        if len(parts) != 2: raise ValueError("Each step needs Temp and Time")
                        temp, mins = float(parts[0]), int(parts[1])
                        
                        # Hardware Safety Check
                        if not (0 <= temp <= 1150): raise ValueError(f"{temp}C is out of range")
                        steps.append((temp, mins))
                        
                    if 0 < len(steps) <= 8: break # Success, move to next prompt
                    else:
                        print(f"Error: You entered {len(steps)} steps. Must be 1 to 8.")
                except (ValueError, IndexError) as e:
                    print(f"Invalid input: {e}")
            
            # --- 3. Flow Logic ---
            while True:
                try:
                    nxt_val = input("Link to Pattern (0-7 or 8 for Stop): ").strip().lower()
                    if nxt_val == 'q': raise InterruptedError
                    nxt = int(nxt_val)
                    if not (0 <= nxt <= 8): raise ValueError("Link must be 0-8")

                    rep_val = input("Repeats (0-99): ").strip().lower()
                    if rep_val == 'q': raise InterruptedError
                    rep = int(rep_val)
                    if not (0 <= rep <= 99): raise ValueError("Repeats must be 0-99")
                    break # Success, move to Review
                except ValueError as e:
                    print(f"Invalid input: {e}")
            
            # --- SAVE THIS PATTERN ---
            New_PID_Program[p_idx] = {"steps": steps, "next": nxt, "repeat": rep}

            # --- ASK TO CONTINUE OR FINISH ---
            more = input("\nAdd/Edit another pattern before uploading? (y/n): ").strip().lower()
            if more == 'q':
                raise InterruptedError  # Jumps to the "User Cancelled" block
            if more != 'y':
                break # Exit outer loop to go to Final Review
        
        # --- Final Review & Upload ---
        if not New_PID_Program:
            raise InterruptedError

        print("\n" + "="*30)
        print("FINAL RECIPE SUMMARY:")
        for p, data in New_PID_Program.items():
            print(f"Pattern {p}: {data['steps']} -> Link: {data['next']}, Rep: {data['repeat']}")
        print("="*30)

        if input("Upload this entire recipe? (y/n): ").strip().lower() == 'y':
            furnace.live_load_profile(New_PID_Program)
            return New_PID_Program
        else:
            raise InterruptedError

    except InterruptedError:
        print("\n[!] User Cancelled. No changes sent. Resuming current program...")
    except Exception as e:
        print(f"\n[ERROR] Unexpected System Error: {e}")
        print("Resuming original program for safety...")
    finally: # Code inside finally will always run.
        # ALWAYS ensure unpause before leaving the function
        furnace.hold(False)
    
def handle_manual_control(furnace, pid_program):
    if msvcrt.kbhit():
        msvcrt.getch() # Clear the key that triggered this
        furnace.hold(active=True) # Pause the Timer
        # Get current hardware status
        status = furnace.get_execution_status() # Expected: (Ptn, Step, TimeStr, Sec)
        current_ptn = status[0] if status else 0
        current_step = status[1] if status else 0
        
        print("\n" + "="*30)
        print(f"{'STATUS':<12} | {'PTN':<4} | {'STEPS (Temp, Min)':<30}") # Formating for display Title
        print("-" * 30)

        for p_idx, data in sorted(pid_program.items()):
            if p_idx < current_ptn:
                label = "[ PAST ]"
            elif p_idx == current_ptn:
                label = "[*ACTIVE*]"
            else:
                label = "[ FUTURE ]"
            
            # --- Format Steps with Active Marker ---
            steps_display = []
            for i, (temp, mins) in enumerate(data['steps']):
                step_text = f"({temp}, {mins})"
                
                # If this is the active pattern AND the active step, add a marker
                if p_idx == current_ptn and i == current_step:
                    # Option A: Arrow pointer
                    steps_display.append(f"--> {step_text}") 
                    # Option B: Brackets (uncomment below if preferred)
                    # steps_display.append(f"[{step_text}]")
                else:
                    # Add spaces to non-active steps so they align with the arrow
                    steps_display.append(f"    {step_text}") 
            
            # Join the steps with a separator
            steps_str = " | ".join(steps_display)
            
            print(f"{label:<12} | {p_idx:<4} | {steps_str}") # Formating for display content
        
        print("="*30)

        print("--- MANUAL CONTROL ---")
        print("1: Skip Step | 2: Load Recipe | 3: STOP Heater | 4: EXIT | 5: Auto Tune")
        print(" (Type 'q' or 'ENTER' to Resume/Cancel)")

        while True:
            choice = input("Selection: ").strip().lower()

            if choice == 'q':
                print("Resuming...")
                furnace.hold(False) # Resume
                return True # Stay in the main logging loop

            if choice == '1':
                furnace.skip_step()
                break # Exit the menu loop
            
            elif choice == '2':
                new_PID_Program = prompt_PID_Program_edit(furnace)
                if new_PID_Program:
                    pid_program.clear()
                    pid_program.update(new_PID_Program)
                break
            
            elif choice == '3':
                furnace.stop()
                print("Heater killed.")
                break
            
            elif choice == '4':
                print("--- STOP_LOGGING ---") # Kill the script
                return False 
            
            elif choice == '5':
                print("--- Entering Auto-Tune Menu ---")
                prompt_auto_tune(furnace)
                furnace.hold(False) # Resume the program timer
                break 
            
            else:
                print(f"\n[!] '{choice}' is not a valid option. Please try again.")
                # The loop continues, re-asking the prompt
            
    return True

# Initialization
furnace = DeltaDTB(port='COM5', baud = 9600) # Check Device Manager/Ports of the cable connection 
temps, times = [], []
start_time = time.time() #saves the start time
test = True
furnace.initialize_hardware(20) # Relay Cycle Set to 20s

# Setup the run
if test:
    # Define the recipe as a simple list
    # [(TargetTemp, Minutes), ...]
    RECYCLE_MODE = False # Set to True to loop forever
    PID_Program = {
        0: {
            "steps": [(50, 10)],
            "next": 1,
            "repeat": 0},
        1: {
            "steps": [(60, 5), (70, 5)],
            "next": 2,
            "repeat": 1},
        2: {"steps": [(25, 20)],
            "next": 8,
            "repeat": 0}
    }
    # Automate the programming
    furnace.load_flow(PID_Program)
    # Start Pattern 0
    furnace.write_register(furnace.START_PATTERN, 0)
    furnace.run()

# Plotting Setup
plt.ion()
fig, ax = plt.subplots()
line, = ax.plot([], [], 'r-') # Create empty line object
ax.set_xlabel("Time (s)")
ax.set_ylabel("Temp (°C)")
ax.grid(True)

with open("furnace_log.csv", "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow(["time_s", "temp_C", "rate_C_min", "P", "I", "D", "Power_Pct", "Ptn", "Step", "TimeStr"])
    
    is_tuning = False
    previous_at_state = False
    keep_logging = True
    while keep_logging: # Persistent wrapper (restarts on 'continue')
        try:
            while keep_logging: # Main execution and telemetry loop for Data acquisition
                is_tuning = furnace.monitor_at_progress()
                if not is_tuning and previous_at_state == True: # Detect the exact moment tuning finishes to resume program mode
                    print("Resuming PID Program Control...")
                    furnace.write_register(furnace.CTRL_METHOD, 3)
                
                if not is_tuning: # Handle Menu (Only if NOT tuning)
                    keep_logging = handle_manual_control(furnace, PID_Program)
                    if not keep_logging: break
                    """
                    # --- SOFTWARE POWER LIMIT (The Governor) ---
                    # Pulse RUN/HOLD to cap the slew rate
                    current_cycle_pos = time.time() % duty_period
                    on_threshold = (power_limit / 100.0) * duty_period
                    
                    if current_cycle_pos < on_threshold:
                        furnace.run()  # 0x103C = 1
                    else:
                        furnace.hold(True) # 0x103C = 3
                    """
                # Only allow manual menu if NOT tuning
                # If handle_manual_control returns False (Choice 4: STOP_LOGGING), keep_logging becomes False
                keep_logging = handle_manual_control(furnace, PID_Program)
                
                # Manual Control Check
                if not keep_logging: break
                
                # Data Acquisition
                temp = furnace.get_pv()
                pid_data = furnace.get_pid_values()
                power_data = furnace.get_power_percent()
                status = furnace.get_execution_status() # (Ptn, Step, TimeStr, Sec)
                
                if temp is not None:
                    cur_time = time.time() - start_time
                    temps.append(temp)
                    times.append(cur_time)
                    
                    rate = furnace.calculate_slew_rate(times, temps)
                    ptn, step, time_str = (status[0], status[1], status[2]) if status else (0,0,"00:00")
                    
                    # Log Data
                    writer.writerow([
                        cur_time, temp, rate, 
                        pid_data[0], pid_data[1], pid_data[2], power_data,
                        ptn, step, time_str
                    ]) #fills out the rows in this order
                    f.flush()

                    print(f"Temp={temp:.1f} °C | Slew={rate:+.2f} °C/min | P={pid_data[0]}, I={pid_data[1]}, D={pid_data[2]} | Power: {power_data}%")
                    
                    # Update Plot
                    line.set_data(times, temps)
                    ax.relim(); ax.autoscale_view()
                    plt.pause(0.01)
                
                previous_at_state = is_tuning
                time.sleep(1)
        except KeyboardInterrupt:
            print("\n[!] User Interrupt Detected. Heater stopped.")
            furnace.stop() # Ensure relay is OFF for shutoff the heating
            
            confirm = input("Heater stopped. Press 'ANY' to exit script, or type 'c' to continue logging cooling: ").strip().lower()
            
            if confirm == 'c':
                print("Logging cooling... Press Ctrl+C again to fully exit.")
                continue # The next iteration of second 'while keep_logging' will continue.
            else:
                keep_logging = False # Disable loop and End
                print("Closing file and port.")
        
        furnace.ser.close()
