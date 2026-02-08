import serial
import time
import csv
import matplotlib.pyplot as plt
import numpy as np
import math
import msvcrt
from collections import deque
from dataclasses import dataclass
import json
import os
from tqdm import tqdm

@dataclass
class SystemConfig:
    # Network Connection (Check Device Manager/Ports of the cable connection)
    port: str = 'COM5'
    baud: int = 9600

    # Timing & Logs
    sample_time_s: float = 1.0          # Global tick for Main Loop Sampling Time
    relay_period_s: float = 20.0        # Relay Cycle Duration in s, default: 20s
    log_history_s: float = 3600.0       # Log time in s to prevent Memory Leak
    
    # Physics & Hardware Limits
    voltage: float = 120.0              # Variac 120/140V
    temp_limit_low: float = 0.0         # Lower Temperature Limit: 0°C
    temp_limit_high: float = 1150.0     # Upper Temperature Limit: 1150°C
    safety_limit: float = 1120.0        # Force Shutdown Temperature: 1120°C
    safety_hys: float = 5.0             # Temperature Hysteresis Setting for Force Shutdown
    max_allowed_rate: float = 35.0      # Force Shutdown Ramp Rate: 35°C/min
    rate_hys: float = 5.0               # Rate Hysteresis Setting for Force Shutdown
    wire_gauge: int = 26                # Kanthal A-1 26g
    wire_density_kg_per_m3: float = 7.1 # Kanthal A-1 Density: 7100kg/m^3
    wire_length_ft: float = 10.0        # 120" Heating Coil Length
    resistance_per_ft: float = 3.2      # Kanthal A-1 26g Resistance: 3.2Ω/ft
    
    # Logic Thresholds
    max_ramp_rate: float = 25.0         # Safe Ramp Rate Limit: 25°C/min
    calibration_window: int = 60        # Sample Size for Calibration before run
    regression_window: int = 3          # Sample Size for Linear Regression (Rate)
    stability_window: int = 30          # Sample Size for Stability Check (Standard Deviation)
    stability_std_dev: float = 0.1      # Standard Deviation Threshold for PID Stability Check
    hys_margin: float = 3.0             # Statistical Significance for Hysteresis band
    rate_snr: float = 1.2               # Statistical Significance for Thermal Response
    coil_safety_margin: float = 60.0    # Safety Limit of Thermal Gradient between Heating Coil and Thermocouple 100°C/cm in 0.60mm ~ 1.28mm (1" OD)
    soak_time_min: float = 2.5          # Initial SOAK Time to split RAMP Mode when Thermal Gradient Safety is triggered
    soak_extension_s: float = 50.0      # Time increment of current SOAK Mode when PID is not stablized
    check_window_multiplier: int = 5    # Number of Attempt to trigger Stability Check in SOAK Mode before it END
    auto_tune_safety: float = 50.0      # Safety Range from Current Temperature for setting Auto-Tune
    
    @property
    def relay_memory_size(self) -> int:
        """Total list size for last relay cycle check."""
        return max(1, math.ceil(self.relay_period_s / self.sample_time_s))
    
    @property
    def data_memory_size(self) -> int:
        """Total list size for the log history."""
        return max(1, math.ceil(self.relay_period_s/self.sample_time_s), math.ceil(self.log_history_s/self.sample_time_s)) # Minimum Relay Cycle Frame or 1
    
    @property
    def stability_window_s(self) -> float:
        """Calculates time window based on multiplier."""
        return self.check_window_multiplier * self.sample_time_s
    
class DeltaDTB: #Hold SET Button, click cycle until seeing Co5H, turn it on to enable Communication Write Enable parameter (Allow modifying setting through python code)
    """
    # Modbus ASCII Format
    # Request Frame (Master to Slave): Colon :(1), Slave ID (2), Function (2), Address (4), Quantity (4), EB- LRC (redundancy check), \r\n- carriage return
    # Response Frame (Slave to Master):: Colon :(1), Slave ID (2), Function (2), Byte Count (2), Data (4), EB- LRC (redundancy check), \r\n- carriage return
    """
    # --- Modbus ASCII Frame Offsets ---
    DATA_START          = 7 # The index where the first register's hex data begins
    HEX_PER_REG         = 4 # Each 16-bit register is represented by 4 hex chars

    # --- Address Map (Class Constants) ---
    # Basic Control Addresses
    UNLOCK              = 0x102C    # Setting lock status Need to set
    HEAT_COOL_SEL       = 0x1006    # Heating/Cooling selection 0: Heating, 1: Cooling, 2: Heating/Cooling, 3: Cooling/Heating
    CTRL_METHOD         = 0x1005    # Control method (PID/Program/Manual) 0: PID, 1: ON/OFF, 2: manual tuning, 3: PID program control
    CYCLE_TIME          = 0x1007    # Relay Cycle: Control cycle of Output 1
    HYS1                = 0x1010    # Hysteresis setting value of the 1st output group 0 ~ 9,999 
    RUN_STOP            = 0x103C    # Run/Stop command  0:Stop, 1:Run, 2:End, 3:Hold

    # Reading
    PV                  = 0x1000    # Process Value Measuring unit is 0.1, updated one time in 0.4 second
    PID_P               = 0x1009    # Proportional Band
    PID_I               = 0x100A    # Integral Time
    PID_D               = 0x100B    # Derivative Time
    OUT1_VOL            = 0x1012    # Power %: On/Off Duration Ratio in Relay Cycle

    # Safety & Limit Addresses
    SENSOR_TYPE         = 0x1004    # CnPt Thermocouple: K, J, T, E, N, R, S, B, L, U, TXK (3-wire Platinum RTD: Pt100, JPt100; Analog input: 0 ~ 5V, 0 ~ 10V, 0 ~ 20 m A, 4 ~ 20 m A, 0 ~ 50mV)
    LIMIT_LOW           = 0x1003    # t-L: Lower-limit of temp range
    LIMIT_HIGH          = 0x1002    # t-H: Upper-limit of temp range
    SV                  = 0x1001    # SV Set Value
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

    def __init__(self, config: SystemConfig, slave_id=1):
        self.cfg = config
        self.ser = serial.Serial(self.cfg.port, self.cfg.baud, bytesize=7, parity='E', stopbits=1, timeout=1)
        self.slave_id = slave_id
    
    @staticmethod
    def _calculate_lrc(hex_string):
        """Calculates the LRC checksum for Modbus ASCII."""
        byte_sum = sum(int(hex_string[i:i+2], 16) for i in range(0, len(hex_string), 2))
        lrc = (0x100 - (byte_sum & 0xFF)) & 0xFF
        return f"{lrc:02X}"
    
    # Write Action
    def _write_register(self, address, value):
        """Sends Function 06 (Write) and verifies the Echo for success/failure."""
        addr_hex = f"{address:04X}"
        val_hex = f"{value:04X}"
        payload = f"{self.slave_id:02X}06{addr_hex}{val_hex}"
        cmd = f":{payload}{self._calculate_lrc(payload)}\r\n".encode('ascii')
        self.ser.write(cmd)
        time.sleep(0.1) # Wait for controller
        resp = self.ser.read(100)
    
        # Fallout Debug Logic
        if not resp:
            print(f"ERROR: No response on write to {hex(address)}")
            return False # Failed Write = False
        elif resp.strip() == cmd.strip(): # Confirmed Received and Executed the command .strip remove \r\n
            print(f"SUCCESS: {hex(address)} set to {value}")
            return True # Sucessful Write = True
        elif b'86' in resp[3:5]: # Exception check :1086 (06 Write Function Code + 80 Modbus SignaL = 86)
            print(f"REJECTED: {hex(address)} Exception Code: {resp[5:7].decode()}")
            return False # Failed Write = False
        else:
            print(f"UNEXPECTED: Response was {resp}")
            return False # Failed Write = False
    
    def safe_write(self, address, value_or_list):
        """Internal helper for 3 silent retries on any write command."""
        is_list = isinstance(value_or_list, list)
        for _ in range(3):
            # Call plural function for lists, singular for single values
            success = self._write_register(address, value_or_list) if is_list else \
                      self._write_register(address, value_or_list)
            if success:
                return True # Sucessful Write = True
            time.sleep(0.1)
        return False # Failed Write = False
    
    # Read Action
    def read_register(self, start_address, count=1):
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
                return None # Failed Read = None
            if not resp.startswith(b':') or not resp.endswith(b'\r\n'): # Basic Modbus ASCII frame validation (starts with : and ends with \r\n)
                print(f"CORRUPTED: Invalid frame from {hex(start_address)}")
                return None # Failed Read = None
            if resp[3:5] == b'83': # Exception check :1083 (03 Read Function Code + 80 Modbus SignaL = 83)
                print(f"REJECTED READ: Exception Code {resp[5:7].decode()}")
                return None # Failed Read = None
            return resp # Successful Read = response message
        
        except serial.SerialException as e:
            print(f"SERIAL ERROR: {e}. Reconnecting...")
            time.sleep(2)
            return None # Failed Read = None
        except Exception as e:
            print(f"INTERNAL DATA ERROR in read_register: {e}")
            return None # Failed Read = None
    
    def _parse_response(self, resp, reg_index=0):
        """
        Extracts the hex string for a specific register index from the response.
        reg_index=0 is the first register, 1 is the second, etc.
        """
        start = self.DATA_START + (reg_index * self.HEX_PER_REG)
        end = start + self.HEX_PER_REG
        return resp[start:end].decode().upper() # Return hex data
    
    # --- Read Data ---
    def get_pv(self):
        """Reads current temperature (Process Value)"""
        resp = self.read_register(self.PV, 1) # Read quantity = 1
        if resp and len(resp) >= 11: # (7 header + (1*4) data + 2 LRC + 2 \r\n)
            try:
                # Extract the hex data
                hex_val = self._parse_response(resp)
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
                    return None # Failed Read = None  
                # Return the temperature if no error
                return val_dec / 10.0 # Return Sensor Temperature(°C)
                
            except (ValueError, UnicodeDecodeError, IndexError):
                print("DEBUG: Process Value Hex Parsing Failed")
        return None # Failed Read = None
    
    def get_sv(self):
        """Reads current temperature (Process Value)"""
        resp = self.read_register(self.SV, 1) # Read quantity = 1
        if resp:
            try:
                # Unit is 0.1°C, so 1000 = 100.0°C
                return int(self._parse_response(resp), 16) / 10.0 # Return Setpoint Temperature(°C) in PID Control Mode(1)
            except:
                print("DEBUG: SV Parsing Failed")
        return None # Failed Read = None
    
    def get_pid_values(self):
        """Reads P, I, and D values (Addresses 1009H - 100BH) in one request using dynamic parsing."""
        resp = self.read_register(self.PID_P, 3) # Read quantity = 3
        if resp:
            try:
                # Indices for multi-register read:
                # Reg1 (P): 7-11 | Reg2 (I): 11-15 | Reg3 (D): 15-19
                p = int(self._parse_response(resp, 0), 16) / 10.0
                i = int(self._parse_response(resp, 1), 16)
                d = int(self._parse_response(resp, 2), 16)
                return p, i, d # Return P, I, D Variable
            except:
                print("DEBUG: PID Parsing Failed")
        return (0.0, 0, 0) # Failed Read = (0,0,0)

    def get_power_percent(self):
        """Reads Output Power % (Address 1012H)"""
        resp = self.read_register(self.OUT1_VOL, 1) # Read quantity = 1
        if resp:
            try:
                # Unit is 0.1%, so 1000 = 100.0%
                return int(self._parse_response(resp), 16) / 10.0 # Return Duty Cycle 0.0 ~ 100.0
            except:
                print("DEBUG: Power Parsing Failed")
        return 0.0 # Failed Read = 0
    
    # --- Initialize Setup ---
    def initialize_hardware(self): # Relay Cycle Time (s) 0 ~ 99, 0:0.5 sec, Temperature Range set from SystemConfig
        """Run once to unlock and set safe physical boundaries."""
        print("--- Initializing Hardware ---")
        self.safe_write(self.UNLOCK, 0)
        # Shut off Heating
        self.safe_write(self.CTRL_METHOD, 2) # Manual Mode (Address 1005H = 2)
        self.safe_write(self.OUT1_VOL, 0) # Set Power to 0 (Address 1012H)
        self.wipe_memory() # Clear Memory
        self.safe_write(self.SENSOR_TYPE, 0) # K-type Thermocouple value is 0
        self.safe_write(self.SV, 0) # Clear the SV to 0°C and prevent limit conflicts
        self.safe_write(self.LIMIT_HIGH, int(self.cfg.temp_limit_high * 10)) # Set Upper limit first to avoid conflict of lower limit, expand the allowable range (Exception Code: 03 Rejected maybe because the existing upper limit is already 1150.0°C.)
        self.safe_write(self.LIMIT_LOW, int(self.cfg.temp_limit_low * 10))
        self.safe_write(self.HEAT_COOL_SEL, 0) # Force Heating mode
        self.safe_write(self.CTRL_METHOD, 3) # PID Program Control Mode (Address 1005H = 3)
        self.safe_write(self.CYCLE_TIME, self.cfg.relay_period_s) # relay_period: The time window for the duty cycle, longer period extends mechanical relay life during pulsing.

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
        self.safe_write(temp_addr, int(temp * 10))
        self.safe_write(time_addr, int(minutes))
    
    def config_pattern_logic(self, pattern, actual_steps, link_to = 8, repeat = 0):
        """
        Configures how a pattern behaves. 1040H to 1047H(Step), 1050H to 1057H(Cycle), 1060H to 1067H(Link)
        actual_steps: 0-7 Select Step Index to end from 1040H to 1047H(Step) of Pattern Index 0 ~ 7
        link_to: 0-7 Select Next Pattern Index to Link or 8 (Stop) from 1050H to 1057H(Cycle) of Pattern Index 0 ~ 7
        repeat = 0 means run once. repeat = 1 means run twice from 1060H to 1067H(Link) of Pattern Index 0 ~ 7
        """
        # Both registers are also contiguous for patterns 0-7
        self.safe_write(self.ACTUAL_STEP_BASE + pattern, actual_steps)
        self.safe_write(self.LINK_PTN_BASE + pattern, link_to)
        self.safe_write(self.PTN_CYCLE_BASE + pattern, repeat)
    
    def load_pattern(self, pattern, profile, next_ptn = 8, repeat = 0, start_point = 0): # Load singular Pattern
        """
        Select Pattern Number 0 ~ 7
        Insert Step Profile: A list of tuples [(temp, mins), (temp, mins), ...] Example: [(100, 10), (100, 30)] -> Ramp to 100 in 10m, Soak at 100 for 30m.
        Select Pattern Link, Number of Repeat, and Step Index for Loading point
        """
        num_steps = len(profile)
        if num_steps > 8:
            raise ValueError(f"Pattern {pattern} exceeded 8 steps.")
        
        # Pattern Steps Profile to selected Pattern Number 0 ~ 7 with selected staring point (Avoid Writing at current operating Step)
        for i in range(start_point, num_steps): # Default starting point = 0 to write the whole Step Profile
            temp, mins = profile[i]
            self.set_pattern_step(pattern, i, temp, mins)

        # Program the pattern flow
        last_steps = num_steps - 1 # Last Step Index 0 ~ 7
        self.config_pattern_logic(pattern, last_steps, next_ptn, repeat) # Set endpoint(Step) of Pattern Index and link to next Pattern after Repeating Number of Selected Pattern Index
        print(f"Pattern {pattern} Loaded: {num_steps - start_point} steps, Next: {next_ptn}, Repeat: {repeat}")
    
    def load_recipe(self, flow_dict): # STOP before loading Master Dictionary
        """
        Load entire PID Program Instruction (Master Dictionary)
        """
        print("--- Loading from Master Dictionary of PID Program ---")
        
        for p_idx, config in flow_dict.items():
            self.load_pattern(
                pattern = p_idx, 
                profile = config["steps"], 
                next_ptn = config.get("next", 8), 
                repeat = config.get("repeat", 0)
            )

        print("--- Loaded Successfully ---")

    def get_execution_status(self):
        """
        Reads the hardware status to see exactly where the furnace is.
        Returns: Pattern, Step, Time_Remaining (Seconds)
        """
        # Read Step and Pattern (Contiguous)
        # 0x1034 (Step), 0x1035 (Pattern)
        pos_data = self.read_register(self.CURR_STEP, 2)
        
        # 2. Read Time Remaining (Separate block at 103DH)
        time_data = self.read_register(self.TIME_REMAIN_SEC, 1)

        if pos_data and time_data:
            try:
                # Parsing based on your parse_response (assuming index refers to register position)
                step    = int(self._parse_response(pos_data, 0), 16)
                pattern = int(self._parse_response(pos_data, 1), 16)
                sec_rem = int(self._parse_response(time_data, 0), 16)
                """
                # Format seconds into MM:SS
                minutes, seconds = divmod(sec_rem, 60)
                time_str = f"{minutes:02d}:{seconds:02d}"
                """
                return pattern, step, sec_rem
            except (ValueError, TypeError, IndexError) as e:
                print(f"DEBUG: Status Parse Error: {e}")
        else:
            print(f"DEBUG: Status Read Failed/Parse Error: (Pos: {len(pos_data) if pos_data else 0}, Time: {len(time_data) if time_data else 0})")
        
        # Return safe defaults if anything fails to prevent script crash
        return 0, 0, 0
        
    def live_load_recipe(self, new_profile_dict): # STOP or HOLD before live loading
        """Overwrite the PID Memory."""
        print("\nINTERRUPT: Loading new profile...")
        try:
            self.load_recipe(new_profile_dict)
            self.safe_write(self.CTRL_METHOD, 3) # PID Program Control Mode (Address 1005H = 3)
            self.safe_write(self.START_PATTERN, 0) # Force the controller to look at Pattern 0, Step 0 immediately
            self.safe_write(self.TIME_REMAIN_SEC, 0) # Resetting Start Pattern (1030H) and clearing current timer (1032H)
            
            self.hold(False) # Release the HOLD and Resume
            print("--- New Profile Active and Jumped to Pattern 0 Step 0 ---")
        except Exception as e:
            print(f"FAILED to load profile: {e}")
            self.hold(False) # Resume so it doesn't stay stuck from menu
    
    
    # --- PID Tuning ---
    def set_tuning(self, p, i, d):
        """Sets the P, I, and D parameters."""
        self.safe_write(self.PID_P, int(p * 10))
        self.safe_write(self.PID_I, int(i))
        self.safe_write(self.PID_D, int(d))
    
    def start_auto_tune(self):
        """Triggers the Auto-Tune (AT) process (0813H)"""
        print("--- Starting Auto-Tune ---")
        # Write 1 to start the tuning process
        return self.safe_write(self.AUTO_TUNE, 1)
    
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
        if self.safe_write(self.RUN_STOP, 1):
            print("Start Running...")
            return True
        print("ERROR: Failed to START after 3 attempts.")
        return False
    
    def stop(self):
        """Hardware Hard Stop (Ends Program, Cuts Power)."""
        if self.safe_write(self.RUN_STOP, 0):
            print("Stop Running...")
            return True
        print("ERROR: Failed to STOP after 3 attempts.")
        return False

    def hold(self, enable_hold=True): # Do NOT recommend because of desync with HeatingCoilModel
        """
        Toggles the 'Hold' state. 
        3: Hold (Pause timer), 1: Run (Resume timer).
        """
        val = 3 if enable_hold else 1 # True: Return 3, False: Return 1
        state_text = "PAUSING (Hold)" if enable_hold else "RESUMING"
        if self.safe_write(self.RUN_STOP, val): # Sucessful Write = True
            print(f"{state_text} Program...")
            return True # Sucessful Write = True
        return False # Failed Write = True
    
    def skip_step(self):
        """Forces the current step to end by setting remaining time to 0."""
        self.run() # Only response to timer when it is in RUN Mode
        return self.safe_write(self.TIME_REMAIN_SEC, 0) # Return Boolean: True(skip); False(failed skip)
    
    def wipe_memory(self):
        """Resets device memory to 0."""
        print("Cleaning PID memory...")
        if not self.stop(): # Prevent Program Running while clearing memory
            print("Warning: Could not verify Stop command before wipe. Proceeding with wipe...")
        time.sleep(0.5) # Wait for hardware
        # Wipe Temperatures (2000H to 203FH = 64 registers) in 4 blocks of 16 (4 * 16 = 64)
        zeros_16 = [0] * 16
        zeros_8 = [0] * 8
        links_8 = [8] * 8 # Set links to 8 (Stop) instead of 0
        for i in range(4):
            start_addr = self.TEMP_BASE + (i * 16)
            self.safe_write(start_addr, zeros_16)
            time.sleep(0.1) # Cooldown for EEPROM
        
        # Wipe Execution Times (2080H to 20BFH = 64 registers) in 4 blocks of 16
        for i in range(4):
            start_addr = self.TIME_BASE + (i * 16)
            self.safe_write(start_addr, zeros_16)
            time.sleep(0.1) # Cooldown for EEPROM
        
        # Wipe Configs: 1040H to 1047H(Step), 1050H to 1057H(Cycle), 1060H to 1067H(Link)
        self.safe_write(self.ACTUAL_STEP_BASE, zeros_8) # Reset Actual Steps to 0
        time.sleep(0.1) # Cooldown for EEPROM
        self.safe_write(self.PTN_CYCLE_BASE, zeros_8) # Reset Cycles to 0
        time.sleep(0.1) # Cooldown for EEPROM
        self.safe_write(self.LINK_PTN_BASE, links_8) # Reset Links to 'End' (8)
        time.sleep(0.1) # Cooldown for EEPROM
        print("Memory Cleared.")
    
    
    
class HeatingCoilModel:
    def __init__(self, config: SystemConfig, start_temp: float = 0.0):
        # Load Setting
        self.cfg = config
        self.current_coil_temp = start_temp # Defaults to 0.0 if not provided
        
        # AWG to Diameter (mm) Formula (ASTM B258) d_n = 0.127 mm * 92^((36-n)/39)
        diameter_mm = 0.127 * 92**((36 - self.cfg.wire_gauge) / 39) # mm
        diameter_m = diameter_mm / 1000 # m
        # Total mass = Area * Length * Density
        area_m2 = np.pi * (diameter_m / 2)**2 # m^2
        self.mass_kg = area_m2 * (self.cfg.wire_length_ft * 0.3048) * self.cfg.wire_density_kg_per_m3 # kg
        # Electrical: Resistance and Power
        self.total_resistance_ohm = self.cfg.wire_length_ft * self.cfg.resistance_per_ft # Ω
        self.max_power_watts = (self.cfg.voltage**2) / self.total_resistance_ohm # Watts (J/s)
        
        # Kanthal A-1 Specific Heat Table (J/kg*K) # Duplicate to cover 0°C
        self.temp_table = np.array([0, 20, 200, 400, 600, 800, 1000, 1200, 1400])
        self.cp_table = np.array([0.46, 0.46, 0.56, 0.63, 0.75, 0.71, 0.72, 0.74, 0.80]) * 1000
    
    def get_kanthal_temperature(self, power_history):
        """
        Only on [RAMP] Mode
        Apply Heating from Duty Cycle and Predict Heating Coil Temperature (Assume no Heat Dissipation)
        Until Reaching Safety Temperature Gradient to Trigger [SOAK] Mode
        Integrates the heating rate over time step 'dt'.
        duty_cycle: 0.0 to 100.0 (from PID)
        """
        # Update Power % Storage of the Relay Cycle Frame
        self.power_check = list(power_history)[-self.cfg.relay_memory_size:] # Return input list if shorter than the window
        # Use the Max Power % of the last Relay Cycle for Safety Limit Overestimation (Relay update at unknown time)
        peak_duty = max(self.power_check, default=0.0) # default=0.0 handles empty input list

        """Calculates instantaneous dT/dt (Degrees/second)"""
        # Interpolate Cp based on current temperature (Linear Piecewise)
        cp = np.interp(self.current_coil_temp, self.temp_table, self.cp_table)
        # Formula: dT/dt = P / (m * Cp)
        heating_rate = (self.max_power_watts * (peak_duty / 100.0)) / (self.mass_kg * cp) # Heating rate (°C/s)
        self.current_coil_temp += heating_rate * self.cfg.sample_time_s
        return self.current_coil_temp
    
    def reset(self, temp): # [!] Assume Heating Coil Temperature = Sensor Temperature after SOAK Mode, which might not be true
        """After [SOAK] Mode Completed, Sync current Temperature to next [RAMP] Mode baseline."""
        self.current_coil_temp = temp # Reset to [RAMP] Mode Initial Temperature after [SOAK] Mode

class PIDProgramManager:
    def __init__(self, furnace, predictor, config: SystemConfig, target_temp: float):
        self.cfg = config
        self.furnace = furnace
        self.predictor = predictor
        self.target = target_temp # Set Target Temperature and Class handle all logic automatically
        
        # Initial Program State
        self.prev_mode = "STARTUP"
        self.prev_tuning_state = False
        self.is_thermal_cutoff, self.triggered_by_rate = False, False
        self.check_cool, self.confirm_cool = False, False
        self.keep_logging = True
        # Menu
        self.in_menu = False
        self.input_active = False
        self.input_sequence, self.input_results = [], []
        self.input_step_idx = 0
        self.input_prompt, self.input_buffer = "", ""
        self.final_callback = None
        # Processing State
        self.ctrl_method = 3            # PID Program Control Mode (Address 1005H = 3)
        self.in_operation_mode = False  # PID Mode (Address 1005H = 0)
        self.in_cooling_mode = False    # Manual Mode (Address 1005H = 2)
        
        # Plotting Setup
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.line, = self.ax.plot([], [], 'r-') # Create empty line object
        self.ax.set_xlabel("Time (s)")
        self.ax.set_ylabel("Temp (°C)")
        self.ax.grid(True)

        # Memory-Safe History Setup
        memory_size = self.cfg.data_memory_size
        self.temp_history, self.time_history, self.rate_history, self.power_history = deque(maxlen=memory_size), deque(maxlen=memory_size), deque(maxlen=memory_size), deque(maxlen=memory_size)
        self.data = { # 1 hour Memory-Safe History
            "time": self.time_history, "temp": self.temp_history,
            "rate": self.rate_history, "power": self.power_history,
            "pid": deque(maxlen=memory_size), "status": deque(maxlen=memory_size) # (P, I, D), (ptn, step, sec)
        }
    
    def calculate_runtime_min(self, original_duration_min, remained_sec):
        return max(1, math.ceil(original_duration_min * 60 - remained_sec) / 60) # Round up to minute, minimum 1 minute

    def calculate_duration_min(self, final_temp, current_temp): # Uses max_ramp_rate from SystemConfig.
        diff = abs(final_temp - current_temp)
        return max(1, math.ceil(diff / self.cfg.max_ramp_rate)) # Minimum 1 minute
    
    def calculate_rate(self):
        """Calculates the rate of change (°/min) using linear regression over the last 'window' data points."""
        # Return 0 during start
        if len(self.temp_history) < 2: return 0.0
        window = min(len(self.temp_history), self.cfg.regression_window) # Number of sampling point for regression
        # Fit a 1st-degree polynomial (line): y = mx + c
        # slope (m) is the first element of the returned array
        x = np.array(list(self.time_history)[-window:])
        y = np.array(list(self.temp_history)[-window:])
        slope, _ = np.polyfit(x, y, 1)
        return slope * 60 # °C/min
    
    def check_stability(self):
        """Check Stability of PID from Temperature Data Frame and Calculate Standard Deviation"""
        recent_data = np.array(list(self.temp_history)[-self.cfg.stability_window:]) # Convert to list() for slicing and np array for Standard Deviation
        variation = np.std(recent_data)
        return variation < self.cfg.stability_std_dev, variation # Return 1. Boolean: True(Stable PID); 2. Float: Standard Deviation
    
    def check_safety(self):
        """Checks if > Safety Limit. Forces 0% power via Manual Mode if unsafe, resumes Program Mode if safe."""
        if self.is_thermal_cutoff:
            if (self.current_temp < (self.cfg.safety_limit - self.cfg.safety_hys) and 
                self.current_rate < (self.cfg.max_allowed_rate - self.cfg.rate_hys)):
                print(f"\n THERMAL SAFE: {self.current_temp:.1f}°C, {self.current_rate:.2f}°C/min. Resuming Program Mode...")
                self.furnace.safe_write(self.furnace.CTRL_METHOD, self.ctrl_method) # Switch back to Original Control Mode (Address 1005H)
                
                if self.triggered_by_rate and self.current_mode == "RAMP":
                    self.path_find() # inject SOAK Mode immediately
                self.is_thermal_cutoff = False
                self.triggered_by_rate = False
                return False  # Proceed in main loop Return Boolean: False(Resume to original PID Program)
            return True # Block main loop, Return Boolean: True(Holding at Manual Mode 0%)
    
        temp_trigger = self.current_temp >= self.cfg.safety_limit # Ignore Heating Coil Model as not in [RAMP] Mode
        rate_trigger = self.current_rate >= self.cfg.max_allowed_rate
        if (temp_trigger or rate_trigger) and not self.is_thermal_cutoff: # Trigger once
            print(f"\n[!] THERMAL SAFETY: Current {self.current_temp:.1f}°C, {self.current_rate:.2f}°C/min | "
                  f"Safety Limit {self.cfg.safety_limit:.1f}°C, {self.cfg.max_allowed_rate:.2f}°C/min")
            print("\nAction: Forcing Manual Mode 0% Output until reaching Safety Temperature.")
            self.e_stop()
            self.is_thermal_cutoff = True
            if rate_trigger:
                self.triggered_by_rate = True
            return True # Block main loop, Return Boolean: True(Holding at Manual Mode 0%)
        return False # No Check trigger and proceed in main loop Return Boolean: False(Resume to original PID Program)
    
    def toggle_cooling(self): # Order: Computation Cheap -> Heavy; Early Exit Order Check -> Exclusive Math
        """Toogle between COOLING State: Queue SOAKING -> Force Stablize -> Forces COOLING Mode -> Resume HEATING (Descending Logic Check)"""
        # Toggle Logic in COOLING Mode
        # Recommend directly using MENU/(2) Load/(A) Full Override to reload PID Program Control Mode(3)
        if self.in_cooling_mode: # [!] Toggle Cautiously
            # PID Control Mode(0)
            if self.ctrl_method == 0:
                next_temp = self.furnace.get_sv()
                msg = f"PID Control Mode(0): Target Temperature {next_temp}°C"
            # PID Program Control Mode(3)
            else: # [Future Development] Safe RAMP Mode Check
                next_temp = self.next_temp
                msg = f"PID Program Control Mode(3): {self.current_mode} Mode: Pattern {self.ptn} Step {self.step}: Target Temperature {next_temp}°C, Duration {self.inital_duration_min}min"
            # Resume not allowed
            if abs(self.current_temp - next_temp) >= self.cfg.auto_tune_safety: # >= Safety Limit from SystemConfig
                print(f"\n[!] SAFETY ERROR: Resuming Temperature ({next_temp}°C) is too far from Current Temperature ({self.current_temp:.1f}°C).")
                print(f"Toggle off COOLING Required Resuming Temperature within {self.cfg.auto_tune_safety}°C for safety.")
                print(f"[TRANSITION] COOLING Stopped. Start Maintaining Current Temperature {self.current_temp:.1f}°C.")
                print("Recommend resume HEATING through MENU/(2) Load/(A) Full Override.")
                self.furnace.safe_write(self.furnace.SV, int(self.current_temp * 10)) # Set current temperature until stable before cooling
                self.furnace.safe_write(self.furnace.CTRL_METHOD, 0) # PID Mode (Address 1005H = 0)
                self.predictor.reset(self.current_temp)
            # Resume allowed
            else: # < Safety Limit from SystemConfig
                self.furnace.safe_write(self.furnace.CTRL_METHOD, self.ctrl_method) # Switch from Manual Control Mode(2) to Original Control Mode (Address 1005H)
                self.predictor.reset(self.current_temp) # Reset HeatingCoilModel as Heating Coil Temperature should be < Current Temperature
                print(f"[RESUME] Cooling cancelled. Returning to Last {msg}")
            self.in_cooling_mode, self.check_cool = False, False # Disable Tag
            return # Early exit
        
        # Auto-Tune Check (skip Queue Check with early exit)
        if self.tuning_state:
            print("[RESUME] Detected Auto-Tune in Progress. COOLING Mode QUEUED. Repeat to END Auto-Tune and PROCEED.")
            self.ctrl_method = 0 # Set Resuming Control Method to PID Mode (Address 1005H = 0)
            if self.check_cool: # Double confirm
                self.skip_action() # Skip Auto-Tune and straight to COOLING Mode
                self.furnace.safe_write(self.furnace.SV, int(self.current_temp * 10)) # Set current temperature until stable before cooling
                self.in_cooling_mode = True # Enter COOLING Mode
                self.check_cool = False # Reset Tag
            self.check_cool = True # Flag 1st confirm
            return # Early exit
        
        # Queue Check (skip Queue Logic with early exit)
        if self.check_cool:
            print("[RESUME] Detected COOLING Queue. Repeat to PROCEED COOLING Immediately.")
            if self.confirm_cool: # Double confirm
                self.e_cool() # Emergency Cooling Logic
                self.in_cooling_mode = True # Enter COOLING Mode
                self.confirm_cool = False # Reset Tag
            self.confirm_cool = True # Flag 1st confirm
            return # Early exit
        
        # Queue Logic
        print("--- COOLING Mode QUEUED: Check Stability before PROCEED ---")
        self.furnace.safe_write(self.furnace.SV, int(self.current_temp * 10)) # Force setpoint to current temperature
        if not self.in_operation_mode or self.current_mode != "SOAK": # Switch to Control Method 0 (PID Mode)
            self.furnace.safe_write(self.furnace.CTRL_METHOD, 0) # PID Mode (Address 1005H = 0)
        print(f"[STABILIZING] Setting to {self.current_temp}°C before cooling...") # Interrupt SOAK and OPERATING Mode to Current Temperature
        self.check_cool = True # Tag a Cooling Logic Check for monitor_execution
    
    def capture_telemetry(self, mode_override): # [!] Place in first of the main loop, Handle non Heating Mode
        """Standardizes data acquisition and prevents memory leaks"""
        try:
            try: # Read Hardware Data
                self.current_temp = self.furnace.get_pv()
                self.current_power = self.furnace.get_power_percent()
                if not mode_override: # Load Current Status if no mode_override
                    self.ptn, self.step, self.sec_rem = self.furnace.get_execution_status()
            except Exception as e: # Update status only if the hardware responds
                print(f"\n[!] Hardware Read Failed - Retaining last known state: {e}")
            
            if mode_override: # Placeholder for mode_override
                self.current_mode = mode_override
            elif self.program and self.ptn in self.program: # Check if self.program contain current Pattern
                try: # Check if current Pattern and Step exist
                    self.current_mode = self.program[self.ptn]["mode"][self.step]
                    self.next_temp, self.inital_duration_min = self.program[self.ptn]["steps"][self.step]
                except (IndexError, KeyError):
                    pass # Keep last known state for out of bounds Pattern/Step
                
            # Time Domain
            self.current_time = time.time() - self.start_time
            self.current_rate = self.calculate_rate()
            # Run Prediction Model when enabled
            if self.current_mode == "RAMP" and self.predictor:
                try:
                    self.coil_temp = self.predictor.get_kanthal_temperature(self.power_history)
                except Exception as e:
                    print(f"Predictor Model Error: {e}")
            else:
                self.coil_temp = None # Irrelevant in non RAMP Mode [!] Beware of Type Error

            # Update self.data and list
            self.time_history.append(self.current_time)
            self.temp_history.append(self.current_temp)
            self.rate_history.append(self.current_rate)
            self.power_history.append(self.current_power)
            self.data["pid"].append((self.kp, self.ki, self.kd)) # Only update after initialization and Auto-Tune, append(tuple) for inner ()
            self.data["status"].append((self.current_mode, self.next_temp, self.ptn, self.step, self.sec_rem, self.coil_temp)) # append(tuple) for inner ()
            return True # Return Boolean

        except Exception as e: # Skip Logging for ANY Failure
            print(f"\n[!] Telemetry Logic Crash: {e}")
            return False # Return Boolean
    
    def calibration(self, pbar, writer, f):
        """Initial baseline capture to determine noise floor before heating."""
        self.initiation_time = time.time()  # Save initialization time
        self.start_time = self.initiation_time + self.cfg.calibration_window * self.cfg.sample_time_s # Predict start_time before run to return negative value data
        self.kp, self.ki, self.kd = self.furnace.get_pid_values() # Initialize PID Variable
        for i in range(self.cfg.calibration_window):
            self.capture_telemetry("CALIBRATION")
            self.save_to_csv(writer, f) # Log to CSV
            
            # Calculate Rolling Noise Floor
            self.temp_noise = np.ptp(self.temp_history) if self.temp_history else 0.0           # Return maximum temperature fluctuation °C
            self.rate_noise = np.max(np.abs(self.rate_history)) if self.rate_history else 0.0   # Return maximum temperature rate fluctuation °C/min 
            
            remained_s = (self.cfg.calibration_window - i) * self.cfg.sample_time_s
            pbar.set_description_str(
                f"[CALIBRATING] Time Remained {remained_s:.1f}s | "
                f"Temp: {self.current_temp:.1f}°C | "
                f"Noise Ceiling: {self.temp_noise:.2f}°C, {self.rate_noise:.2f}°C/min"
            )
            time.sleep(self.cfg.sample_time_s)
        
        # Initialization
        self.start_time = time.time() # Set Start time for actual run
        self.hysteresis = self.temp_noise * self.cfg.hys_margin # SystemConfig Multiplier for Statistical Significance
        self.furnace.safe_write(self.furnace.HYS1, int(self.hysteresis * 10)) # Address (1010H): Round to 0.1°C
        self.noise_ceiling = self.rate_noise * self.cfg.rate_snr # SystemConfig Multiplier for Statistical Significance
        print(f"\n[READY] Noise Ceiling locked at: {self.temp_noise:.2f}°C, {self.rate_noise:.2f}°C/min | "
              f"Set: Hysteresis {self.hysteresis:.2f}°C, Noise Ceiling: {self.noise_ceiling}")
        
        # Loading Setup
        self.program = self.generate_initial_program(self.target, self.current_temp) # Initialize PID Program
        self.furnace.load_recipe(self.program)  # Load initial PID Program
        self.furnace.safe_write(furnace.START_PATTERN, 0) # Start at Pattern 0 Step 0
        self.furnace.run() # Start run and proceed to main loop
        self.predictor.reset(self.current_temp) # Reset Heating Coil Model
        print(f"\n[RUN] Hardware Started. Entering Main Loop... | Current Temperature {self.current_temp}°C.")
        self.is_measuring_lag = True
    
    def generate_initial_program(self, final_temp, initial_temp): 
        """Initial Guess of RAMP Mode to Target Temperature, Subjected to Refinement"""
        duration = self.calculate_duration_min(final_temp, initial_temp)
        return {
            0: { # Pattern Index 0 ~ 7
                "mode": ["RAMP"], # Mode Available: ["RAMP"] ["SOAK"] ["OPERATING"] ["COOL"]
                "steps": [(final_temp, duration)], # [(°C, min)] Tuple in List
                "next": 8, # Link to next Pattern 0 ~ 7, 8 Stop
                "repeat": 0 # Cycle Number 0 ~ 99
            }
        }
    
    def path_find(self): # only handle current event
        """
        Refine PID program and Update Internal Dictionary:
        1. Updating the Current RAMP's duration.
        2. Inserting SOAK immediately.
        3. Adding the Next RAMP.
        """
        if self.current_mode != "RAMP":
            print("Selected Mode is not [RAMP]! No Action Taken!")
            return self.program # Stop the code below
        
        # Dissect RAMP to Old: RAMP and New: SOAK RAMP [(°C, min)] Tuple in List
        orig_ramp = (self.current_temp, self.calculate_runtime_min(self.inital_duration_min, self.sec_rem))
        soak_step = (self.current_temp, self.cfg.soak_time_min) # SystemConfig initial [SOAK] time
        next_ramp = (self.next_temp, self.calculate_duration_min(self.target, self.current_temp)) 
        
        # Temporary holder list for splicing extra (>8 steps) to next Pattern
        combined_steps = self.program[self.ptn]["steps"][:self.step] + [orig_ramp, soak_step, next_ramp]
        combined_modes = self.program[self.ptn]["mode"][:self.step] + ["RAMP", "SOAK", "RAMP"]

        if len(combined_steps) <= 8: # Check Room for Pattern and Update current Pattern
            self.program[self.ptn]["steps"], self.program[self.ptn]["mode"] = combined_steps, combined_modes
            self.furnace.load_pattern(self.ptn, combined_steps, next_ptn=self.program[self.ptn]["next"], start_point=self.step)
        else: # Fill up current Pattern, Move extras to next Pattern
            target_next_ptn  = (self.ptn + 1) % 8 # Circular Wrap, will overwrite Pattern 0 if overflow
            remained_modes = combined_modes[:8]
            remained_steps = combined_steps[:8]
            leftover_modes = combined_modes[8:]
            leftover_steps = combined_steps[8:]

            # Handle current Pattern
            self.program[self.ptn]["steps"], self.program[self.ptn]["mode"] = remained_modes, remained_steps
            self.program[self.ptn]["next"] = target_next_ptn # Link to next Pattern
            self.furnace.load_pattern(self.ptn, remained_steps, next_ptn=target_next_ptn, start_point=self.step)
            
            # Handle new Pattern
            self.program[target_next_ptn] = { # Create next Pattern in PID Program Dictionary
                "mode": leftover_modes,
                "steps": leftover_steps,
                "next": 8,
                "repeat": 0
            }
            self.furnace.load_pattern(target_next_ptn , leftover_steps, next_ptn=8) # start_point = 0, [!] does not wipe entire Pattern legacy Step beyond leftover_step
            self.furnace.skip_step()
            
        return self.program
    
    def skip_action(self): # Skip cautiously for HeatingCoilModel
        """Handles 'Skip' logic for both Program Mode and Auto-Tune Mode. Ignore in OPERATING and COOLING Mode"""
        if self.tuning_state: # Skip Auto-Tune and return back to Control Mode(3): PID Program
            self.furnace.safe_write(self.furnace.AUTO_TUNE, 0) # Stop the AT process (Write 0 to 0813H)
            time.sleep(0.5) # Wait for hardware
            self.at_complete()
            print(f"[ABORT] Cancelling Auto-Tune. Recovering partial PID: Kp {self.kp}, Ki {self.ki}, Kd {self.kd} | Resuming {self.current_mode} Mode: Pattern {self.ptn} Step {self.step}: Target Temperature {self.next_temp}°C, Duration {self.inital_duration_min}min")
        else: # Skip current Step
            if self.ctrl_method == 3: # Only allow skip Step in PID Program Control Mode (Address 1005H = 3)
                self.furnace.skip_step()
                print(f"Skipping {self.current_mode} Mode: Pattern {self.ptn} Step {self.step}: Target Temperature {self.next_temp}°C, Duration {self.inital_duration_min}min")
    
    def at_complete(self): # Transit from Auto-Tune and switch to Control Method 3 (PID Program Control)
        self.furnace.safe_write(self.furnace.CTRL_METHOD, self.ctrl_method) # Switch back to Original Control Mode (Address 1005H)
        self.predictor.reset(self.current_temp) # Reset HeatingCoilModel from finishing SOAK Mode during Auto-Tune
        self.kp, self.ki, self.kd = self.furnace.get_pid_values() # Update PID Variable after Auto-Tune
    
    def cooling(self): # Switch to Control Method 2 (Manual Mode) and Set Duty Cycle to 0% gradually
        """Switch to"""

    def e_stop(self): # Switch to Control Method 2 (Manual Mode) and Set Duty Cycle to 0%
        self.furnace.safe_write(self.furnace.CTRL_METHOD, 2) # Manual Mode (Address 1005H = 2)
        self.furnace.safe_write(self.furnace.OUT1_VOL, 0) # Set Duty Cycle 0% (Address 1012H)
    
    def operation_mode(self):
        print(f"--- Operation Mode ENABLED: {self.current_temp}°C ---")
        # Save last Mode actual duration
        self.program[self.ptn]["steps"][self.step] = (self.current_temp, self.calculate_runtime_min(self.inital_duration_min, self.sec_rem)) # Set Runtime of last Mode reaching Target Temperature (Tuple is immutable)
        # Set target temperature setpoint and switch to Control Method 0 (PID Mode)
        self.ctrl_method = 0 # Save Current Control Method 0 (PID Mode)
        self.furnace.safe_write(self.furnace.SV, int(self.target * 10))
        self.furnace.safe_write(self.furnace.CTRL_METHOD, self.ctrl_method) # PID Mode (Address 1005H = 0)
        self.in_operation_mode = True
        # Log PID Program History
        PID_recipe = self.program
        self.save_recipe_to_disk(PID_recipe, filename=f"run_instruction_{int(time.time())}.json")
    """
        ### Legacy Logic: Reversed Heating and Pattern 7 reserved for Operation Mode ###
        cooling_recipe = self.generate_cooling_profile(PID_recipe)
        master_instruction = {
            "heating_history": PID_recipe,
            "cooling_mirror": cooling_recipe
        }
        self.save_recipe_to_disk(master_instruction, filename=f"run_instruction_{int(time.time())}.json")
        # Declare Pattern for Operation
        op_ptn = 7 # Place at the End of Pattern WiP: op_ptn = (ptn + 1) % 8 and dynamic load of PID_recipe exceed 56 steps
        
        # Create the New Pattern (Step extend infinitely until interrupted)
        op_steps = [(self.target, 60.0)] # 60min Runtime
        self.program[op_ptn] = { # Create next Pattern in PID Program Dictionary
            "mode": ["OPERATION"],
            "steps": op_steps,
            "next": 0,  # Link to Cooling Mode
            "repeat": 0
        }
        
        # Switch to Operation Mode
        self.furnace.load_pattern(self.ptn, self.step, next_ptn=op_ptn) # Update Pattern and Linking of current Mode Pattern
        self.furnace.skip_step() # Terminate last mode
        self.furnace.load_pattern(op_ptn, op_steps, next_ptn=0) # Update Pattern and Linking to Operation Mode Pattern
        self.in_operation_mode = True
        self.distribute_load(start_ptn=0, 
                        flat_steps=[s[0] for s in cooling_recipe], 
                        flat_durations=[s[1] for s in cooling_recipe], # Assuming distribute_load takes both
                        flat_modes=["COOL"] * len(cooling_recipe), 
                        final_link=8)
        
    def generate_cooling_profile(self, pid_recipe):
        # Reverses the PID dictionary into a flat list of (Target, Duration).
        # Ensures the cooling starts from the current temp and ends at the start temp.
        # Flatten all PID Steps into a single list
        pid_step = []
        for p_idx in sorted(pid_recipe.keys()):
            if p_idx == 7: continue # Skip Operation Mode as it is running
            pid_step.extend(pid_recipe[p_idx]["steps"]) # Append Pattern 0 ~ 6 to pid_steps list
        
        # Extract temps and durations
        pid_temp = [s[0] for s in pid_step]
        pid_duration = [s[1] for s in pid_step]
        
        # Create Cooling Targets using the previous targets as the new goals.
        cooling_target = pid_temp[:-1][::-1] # [:-1] Remove last Step reach to target Temperature [::-1] Reverse the list
        # Append shutdown Temperature
        shutdown_temp = pid_temp[0] if pid_temp else 25.0 # Safety Fallback if temps[0] is empty
        cooling_target.append(shutdown_temp)
        
        # Retrieve Safety Reverse Duration
        cooling_duration = []
        mirrored_duration = pid_duration[::-1] # Reverse the list
        
        starting_temp = [self.target] + cooling_target[:-1] # Append target Temperature at the start
        
        for i in range(len(cooling_target)):
            target_temperature = cooling_target[i]
            start_temperature = starting_temp[i]
            
            # Calculate engineering safety time for THIS specific drop
            safe_time = self.calculate_duration_min(target_temperature, start_temperature)
            
            # Use the longer of the mirrored time OR the safe time
            cooling_duration.append(max(mirrored_duration[i], safe_time))
        
        cooling_profile = list(zip(cooling_target, cooling_duration))
        return cooling_profile
    """
    def safe_inject(self, new_steps, target_ptn, target_step): # [!] target MUST be in Future (> Current Pattern) to avoid interferring current running Step
        """
        Safely injects steps by moving the Memory after injection point to a new pattern index.
        Preserves all gaps, links, and steps in original PID Program.
        [1] | [2]
        [3]
        ->
        [1]
        [new]
        [2]
        [3]
        """
        print(f"--- Safe Injection: Inserting at Pattern {target_ptn} Step {target_step} ---")

        # Save tail [] |* []*, selected Pattern after the injection point of the selected Pattern
        tail_steps = self.program[target_ptn]["steps"][target_step + 1:]
        tail_modes = self.program[target_ptn]["mode"][target_step + 1:]
        original_link = self.program[target_ptn]["next"]

        # Save head []* | [], select selected Pattern up till injection point so it becomes the Endpoint of the selected Pattern
        head_steps = self.program[target_ptn]["steps"][:target_step + 1]
        head_modes = self.program[target_ptn]["mode"][:target_step + 1]
        
        # Store every Pattern after injection point
        ptn_chain = []
        curr = target_ptn + 1
        while curr in self.program and curr != 8: # [!] Only compatible with Ascending Pattern Link with no Repeat
            ptn_chain.append(curr) # Store the Pattern Link without handling Repeat
            curr = self.program[curr]["next"] # Check what is the next Pattern Link iteratively
        
        # Reserve room for New Pattern for the insert Program (flatten list)
        new_patterns_needed = math.ceil(len(new_steps) / 8)
        # Logic to arrange tail (if not empty reserve 1 more pattern to put the tail)
        tail_block_needed = 1 if tail_steps else 0
        total_shift = new_patterns_needed + tail_block_needed

        # Handle Future Pattern
        # Descending order to avoid overwriting
        for ptn_idx in reversed(ptn_chain):
            new_idx = (ptn_idx + total_shift) % 8  # Circular Wrap, will overwrite Pattern 0 if overflow
            self.program[new_idx] = self.program.pop(ptn_idx) # Ctrl X the last pattern to new location
            # Update internal links to follow the shift
            if self.program[new_idx]["next"] != 8:
                self.program[new_idx]["next"] = (self.program[new_idx]["next"] + total_shift) % 8
                
            # Update Pattern and Linking after injection point to new location
            self.furnace.load_pattern(new_idx, self.program[new_idx]["steps"], 
                                    next_ptn=self.program[new_idx]["next"])
        
        # Handle head
        # Link selected Pattern to New Pattern
        first_new_ptn = (target_ptn + 1) % 8 # Connect selected Pattern to the first New Pattern
        self.program[target_ptn]["steps"] = head_steps
        self.program[target_ptn]["mode"] = head_modes
        self.program[target_ptn]["next"] = first_new_ptn

        # Handle tail if it exists
        # Declare tail Pattern Index
        tail_ptn_idx = (first_new_ptn + new_patterns_needed) % 8 # Circular Wrap, link to Pattern 0 if overflow
        # Link inserted Pattern to tail if it exist
        target_link_for_new = tail_ptn_idx if tail_steps else original_link
        # Update Dictionary if it exist
        if tail_steps:
            self.program[tail_ptn_idx] = {
                "mode": tail_modes, "steps": tail_steps,
                "next": original_link, "repeat": 0
            }
            # Update Pattern and Linking of tail to selected Pattern original Link
            self.furnace.load_pattern(tail_ptn_idx, tail_steps, next_ptn=original_link)
        # Update Hardware Linking of selected Pattern to New Pattern
        self.furnace.load_pattern(target_ptn, head_steps, next_ptn=first_new_ptn)  # [!] Did not wipe Step Memory but will not cause issue as it End at insertion step
        
        # Arrange flatten list to New Pattern linked to tail Pattern or original link and load to Dictionary and hardware
        self.distribute_load(first_new_ptn, new_steps, final_link=target_link_for_new)

    def unsafe_inject(self, new_steps, new_modes, target_ptn, target_step, override=False):
        """
        Splicing logic that respects existing pattern boundaries.
        """
        # Select Pattern, Step and Mode
        current_steps = self.program[target_ptn]["steps"]
        current_modes = self.program[target_ptn]["mode"]

        # Create the new sequence for Selected Pattern
        if override:
            # [History before Insertion Point] + [New Steps]
            updated_steps = current_steps[:target_step] + new_steps
            updated_modes = current_modes[:target_step] + new_modes
        else: #[!] Does not handle Pattern data after selected Pattern (Overwrite it)
            # [History before Insertion Point] + [New Steps] + [History after Insertion Point]
            updated_steps = current_steps[:target_step] + new_steps + current_steps[target_step:]
            updated_modes = current_modes[:target_step] + new_modes + current_modes[target_step:]

        # Handle Pattern Overflow
        # Chunk it into 8-step blocks starting at target_ptn
        idx = target_ptn
        while updated_steps:
            chunk_steps = updated_steps[:8]
            chunk_modes = updated_modes[:8]
            
            # Chuck length >8, link to the next Pattern
            # Otherwise, we keep the original 'next' link from the recipe
            if len(updated_steps) > 8:
                next_val = idx + 1
            else:
                # If it's the last chunk, keep the user's original link (e.g., 8 or a specific jump)
                next_val = self.program.get(idx, {}).get("next", 8)

            # Update Local Dictionary
            self.program[idx] = {
                "steps": chunk_steps,
                "mode": chunk_modes,
                "next": next_val,
                "repeat": 0
            }

            # Load Pattern after insertion point
            start_write = target_step if idx == target_ptn else 0 # Return selected Step if in selected Pattern, otherwise Return Step 0 for ongoing Pattern
            self.furnace.load_pattern(idx, chunk_steps, next_ptn=next_val, start_point=start_write)
            # Remove the 8 we just processed
            updated_steps = updated_steps[8:]
            updated_modes = updated_modes[8:]
            
            # If we pushed data into a NEW pattern, we continue the loop
            if updated_steps:
                idx += 1
                # If the next pattern already existed, we must 'flatten' its 
                # data into our current stream so we don't overwrite it
                if idx in self.program and not override:
                    updated_steps += self.program[idx]["steps"]
                    updated_modes += self.program[idx]["mode"]

        print(f"Injection complete. Handled cascade up to Pattern {idx}")

    def distribute_load(self, start_ptn, flat_steps, final_link=8):
        """Distributes raw PID Program (list of tuples: [(temp, mins), (temp, mins), ...]) across Pattern 0 ~ 7 starting at Pattern Index"""
        total_steps = len(flat_steps)
        curr_ptn = start_ptn
        prev_ptn = (start_ptn - 1) % 8 # [!] Assume circular ascending Linking
        try:
            prev_temp = self.program[prev_ptn]["steps"][-1][0] # last step of the previous pattern
        except:
            print("No link to LOAD Pattern! Continue loading Pattern.")
            prev_temp = None
        # Process steps in chunks of 8
        for i in range(0, total_steps, 8): # i = interval of 8 (0, 8, 16, ...)
            # Slice the next 8 steps and modes
            chunk_s = flat_steps[i : i + 8]
            chunk_m = [] # Reset every chunk interval
            for next_temp, _ in chunk_s:
                # Automatically detects RAMP/SOAK modes
                mode = "SOAK" if next_temp == prev_temp else "RAMP"
                chunk_m.append(mode)
                prev_temp = next_temp # Update for next step comparison
            
            # Setup linking
            if i + 8 < total_steps: # Iterate until reaching End of flatten list
                next_link = (curr_ptn + 1) % 8 # Circular Wrap, link to Pattern 0 if overflow
            else: # Link the final chunk
                next_link = final_link

            # Update Internal Dictionary
            self.program[curr_ptn] = {
                "steps": chunk_s, 
                "mode": chunk_m, 
                "next": next_link, 
                "repeat": 0
            }

            # Update Pattern and Linking to Hardware
            self.furnace.load_pattern(curr_ptn, chunk_s, next_ptn=next_link)
            
            # Move to next Pattern index for next iteration
            curr_ptn = (curr_ptn + 1) % 8 # Circular Wrap, link to Pattern 0 if overflow
        
    def monitor_execution(self): # Order: Frequent -> Rare; Computation Cheap -> Heavy; Global -> Sync -> Exclusive Math (Early Exit)
        """Decision making based on current status"""
        # Global Check
        # Thermal and Rate Safety Check
        if self.check_safety(): return

        # Cooling Check
        if self.check_cool:
            is_stable, current_std = self.check_stability()
            if is_stable:
                self.cooling()
                self.in_cooling_mode = True
                self.check_cool, self.confirm_cool = False, False
            return # Early exit
        
        # Auto-Tune Check
        self.tuning_state = self.furnace.monitor_at_progress() # Boolean: True(Autotuning); False(Done)
        if not self.tuning_state and self.prev_tuning_state: # Current State False and Previous State True when done AutoTune
            self.at_complete()
            print(f"\n[TRANSITION] Auto-Tuning Complete. Kp {self.kp}, Ki {self.ki}, Kd {self.kd} | Resuming {self.current_mode} Mode: Pattern {self.ptn} Step {self.step}: Target Temperature {self.next_temp}°C, Duration {self.inital_duration_min}min")
        self.prev_tuning_state = self.tuning_state
        
        # Thermal Response Detection (Lag Time)
        if self.is_measuring_lag and self.current_rate > self.noise_ceiling: # Only for initial heating to overestimate lag time
            self.lag_time = time.time() - self.start_time
            self.is_measuring_lag = False # Detection complete
            print(f"\n[THERMAL RESPONSE] Lag Time: {self.lag_time:.1f}s")
        
        # Target Temperature Check
        if not self.in_operation_mode and self.current_temp >= self.target:
            self.operation_mode()
        
        # Transition Check
        if self.current_mode != self.prev_mode:
            print(f"Entering {self.current_mode} Mode: Pattern {self.ptn} Step {self.step}: Target Temperature {self.next_temp}°C, Duration {self.inital_duration_min}min")
            
            # Sync coil for next RAMP
            if self.current_mode == "RAMP":
                self.predictor.reset(self.current_temp) 
                self.coil_temp = self.current_temp # Initialize first Heating Coil Temperature Datapoint
            
            self.prev_mode = self.current_mode
        
        # Exclusive Check
        # RAMP Check
        if self.current_mode == "RAMP":
            # Run the Heating Coil/Thermocouple Thermal Gradient Check
            if self.coil_temp > (self.current_temp + self.cfg.coil_safety_margin): # Only Dissect Ramp when Thermal Gradient Threshold Exceed
                print(f"\n[SOAK] Mode Triggered: Predicted Coil {self.coil_temp:.1f}°C > Thermocouple {self.current_temp:.1f}°C + {self.cfg.coil_safety_margin}")
                self.path_find()
            return # Early exit
        
        # SOAK Check
        elif self.current_mode == "SOAK":
            # Run the Stability Check
            if self.sec_rem <= self.cfg.stability_window_s: # Check Stability before it END by SystemConfig of Attempt
                is_stable, current_std = self.check_stability()
                if not is_stable: # Only extend when Stability Check Failed
                    print(f"\nStability Requirement Failed σ = {current_std:.3f} < {self.cfg.stability_std_dev:.2f} (S.D. Threshold) | Extending SOAK Mode by {self.cfg.soak_extension_s}s...")
                    self.furnace.safe_write(self.furnace.TIME_REMAIN_SEC, self.sec_rem + self.cfg.soak_extension_s)
                    # Update dictionary
                    self.program[self.ptn]["steps"][self.step] = (self.current_temp, self.inital_duration_min + (self.cfg.soak_extension_s/60))
            return # Early exit
    
    def run_main_loop(self):
        """The persistent main execution loop."""
        print("--- Starting PID Program Manager ---")
        # Setup Hardware
        self.furnace.initialize_hardware()
        # Create csv file
        with open("furnace_log.csv", "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["Time(s)", "Temp(°C)", "Rate(°C/min)", "P", "I", "D", "Power(%)", "Current Mode", "Next Temperature Target(°C)", "Pattern index", "Step index", "Remained Time(s)", "Predicted Heating Coil Temperature(°C)"])
            
            # Initialize dashboard for live display
            with tqdm(total=0, bar_format='{desc}', mininterval=0) as pbar: # total=0 disable % bar, bar_format='{desc}' only show text, mininterval=0 update instantly
                # Calibrate before run (negative time)
                self.calibration(pbar, writer, f)
                # Main loop
                while self.keep_logging:
                    try: # Order: Data -> Status -> Action
                        if not self.capture_telemetry():
                            continue # Skip if capture_telemetry failed
                        
                        # Live Display
                        dashboard = (
                            f"Time={self.current_time:.0f}s | "
                            f"Temp={self.current_temp:.1f} °C | "
                            f"Rate={self.current_rate:+.2f} °C/min | "
                            f"Power={self.current_power}% | "
                            f"P={self.kp}, I={self.ki}, D={self.kd} | "
                            f"Mode={self.current_mode} T_Target={self.next_temp}°C | "
                            f"Pattern={self.ptn} Step={self.step} Time Remained={self.sec_rem}s | "
                            f"T_Coil={f'{self.coil_temp:.1f}°C' if self.coil_temp is not None else 'N/A'} "
                        )
                        pbar.set_description_str(dashboard)

                        # Manual Menu (Non-blocking)
                        if not self.handle_manual_control(pbar):
                            break # User chose EXIT (Option 4)
                        
                        # Decision Logic
                        self.monitor_execution()
                        
                        # Write to CSV
                        self.save_to_csv(writer, f)
                        
                        # Update Visualization (Optional)
                        self.update_plot()
                        
                        time.sleep(self.cfg.sample_time_s)
                    
                    except KeyboardInterrupt: # Trigger by Ctrl+C
                        print("\n[!] User Interrupt Detected. Enter Cooling Mode!")
                        self.in_cooling_mode = True # Enter cooling mode instead of hard shut off
                        break
                
    def handle_manual_control(self, pbar): # Order: Nested to Top level
        """Non-blocking menu: Keys 1-5 only work after pressing 'M'."""
        if not msvcrt.kbhit(): return # Exit when no key pressed
        while msvcrt.kbhit(): # Empty the keyboard buffer completely every cycle
            key = msvcrt.getch().decode().lower() # Clear the key that triggered this
            
            # 3rd Level: Typing Data (Non-blocking Input)
            if self.input_active:
                self.process_typing(key, pbar)
                return 

            # 2nd Level: Sub-Menu
            if self.menu_level == 'LOAD_FORK':
                self.process_load_fork(key, pbar)
                return 

            # 1st Level: Main Menu
            if self.in_menu:
                self.process_main_commands(key, pbar)
            
            # Idle
            elif key == 'm':
                self.enter_main_menu(pbar)
        
    def enter_main_menu(self, pbar): # Idle
        self.in_menu = True
        self.menu_level = 'MAIN'
        # Live Menu
        pbar.write("\n" + "="*30)
        pbar.write(f"{'STATUS':<12} | {'PTN':<4} | {'STEPS (Temperature °C, Duration min)':<50}")
        pbar.write("-" * 30)
        # Sort Patterns in ascending order [!] ignore circular wrap
        for p_idx, data in sorted(self.program.items()):
            if p_idx < self.ptn:
                label = "[  PAST  ]"
            elif p_idx == self.ptn:
                label = "[*ACTIVE*]"
            else:
                label = "[ FUTURE ]"
            # Initiate Step list for Pattern loop
            steps_list = []
            for i, (temp, mins) in enumerate(data['steps']):
                step_str = f"({temp}°C, {mins}min)" # Create string for each step
                # Mark current step
                if p_idx == self.ptn and i == self.step:
                    steps_list.append(f"-> {i}:{step_str}")
                else:
                    steps_list.append(f"{i}:{step_str}")
            # Concatenate Step list and display in menu
            formatted_steps = " | ".join(steps_list)
            pbar.write(f"{label:<12} | {p_idx:<4} | {formatted_steps}")
        pbar.write("="*30)
        pbar.write("\n--- [MAIN MENU] (1) Skip | (2) Load | (3) Tune | (4) Toggle (Cool/Cancel) | (5) E-STOP | (ANY) Close Menu and Resume ---")
    
    def process_main_commands(self, key, pbar): # 1st Level Menu
        """Processes top-level hotkeys."""
        if key == '1':
            self.skip_action()
        elif key == '2': 
            self.menu_level = 'LOAD_FORK'
            pbar.write("[LOAD MENU] (A) Full Override | (B) Safe Inject | (ANY) Quit Load Menu")
        elif key == '3': 
            self.setup_input("[PROMPT] Enter Target Temperature for Auto-Tune: ", self.callback_at, pbar)
        elif key == '4':
            self.toggle_cooling()
        elif key == "5":
            self.e_stop() # Safety Guarantee Stop Logic
            self.furnace.stop()
            print("[STOP] Heater Killed.")
        else: # Any key other than 1-5 closes the menu
            self.in_menu = False
            pbar.write("[RESUME] Menu Closed.")
    
    def process_load_fork(self, key, pbar): # 2nd Level Menu
        """Handles the sub-menu choices for Pattern Loading."""
        if key == 'a': # (A) Full Override
            self.temp_recipe_dict = {} # Temporary storage for building the recipe
            prompts = ["Pattern (0-7): ", "Steps (°C, min; °C, min): ", "Link (0-8): ", "Repeat (0-99): ", "Add another? (y/n): "]
            self.setup_input(prompts, self.callback_live_recipe, pbar)
        elif key == 'b': # (B) Safe Inject
            prompts = ["Steps (°C, min; °C, min): ", "Selected Pattern for insert: ", "Selected Step for insert: "]
            self.setup_input(prompts, self.callback_safe_inject, pbar)
        else:
            self.menu_level = 'MAIN'
            pbar.write("[BACK] Main Menu.")
    
    def process_typing(self, key, pbar): # 3rd Level Menu
        """Handles logic for live character input."""
        if key == '\r': # Enter
            # Save the current answer
            self.input_results.append(self.input_buffer)
            self.input_step_idx += 1
            # Loop all of the questions
            if self.input_step_idx < len(self.input_sequence):
                self.input_buffer = ""
                self.input_prompt = self.input_sequence[self.input_step_idx]
            else: # Complete all questions, Run the command
                self.final_callback(self.input_results, pbar)
                self.input_active = False
        elif key == 'q': # 'q': Global Abort
            self.input_active = False
            pbar.write("[CANCEL] Input mode exited.")
        elif key == '\x08': # Backspace
            self.input_buffer = self.input_buffer[:-1]
        elif key in "0123456789.;,- yno":
            self.input_buffer += key
        
        pbar.set_description_str(f"[{self.input_prompt}]: {self.input_buffer}_")

    def setup_input(self, prompts, final_callback, pbar): # Live typing to insert input to callback function
        """Triggers the input buffer state for sequence of questions."""
        self.input_active = True
        # If user passed a single string, wrap it in a list
        self.input_sequence = [prompts] if isinstance(prompts, str) else prompts # e.g., ["Steps?", "Pattern?", "Step?"]
        self.input_step_idx = 0
        self.input_results = []
        self.final_callback = final_callback
        self.input_buffer = ""
        self.input_prompt = self.input_sequence[0]
    
    def callback_live_recipe(self, answers, pbar, current_build=None): # Called after finish input for live_load_recipe in class DeltaDTB
        """
        answers[0]: Selected Pattern index 0 ~ 7
        answers[1]: Step content [°C, min; °C, min] convert to [(°C, min)] Tuple in List
        answers[2]: Linked Pattern index 0 ~ 7
        answers[2]: Number of Repeat 0 ~ 99 (Actual runtime +1)
        """
        if current_build is None: current_build = {} # Initialzie placeholder for PID Program
        try:
            p_idx = int(answers[0])
            step_content = []
            for s in answers[1].strip(';').split(';'):
                t, m = s.split(',')
                step_content.append((int(t), int(m))) # append(tuple) for inner (), Round to integer
            
            # Store in the dictionary and pass iteratively
            current_build[p_idx] = {
                "steps": step_content, 
                "next": int(answers[2]), 
                "repeat": int(answers[3])
            }

            if answers[4].lower() == 'y': # 'y': re-trigger to add another pattern
                prompts = ["Pattern (0-7): ", "Steps (°C,min; °C,min): ", "Link (0-8): ", "Repeat (0-99): ", "Add another? (y/n): "]
                self.setup_input(prompts, lambda res, pb: self.callback_live_recipe(res, pb, current_build), pbar)
            else: # 'any': cancel and display review
                pbar.write("\n" + "="*20 + " REVIEW RECIPE " + "="*20)
                for p, data in sorted(current_build.items()):
                    pbar.write(f" Pattern {p}: {data['steps']} -> Next: {data['next']}, Repeat: {data['repeat']}")
                pbar.write("="*55)
                
                # Final Confirmation: Only 'y' proceeds, anything else cancels
                self.setup_input("Upload to Live Load Recipe? (y/any): ", 
                                 lambda res, pb: self.final_upload_confirm(res, pb, current_build), pbar)
                
        except Exception as e:
            pbar.write(f"[ERROR] Live Load Recipe failed: {e}. Restarting recipe build.")

    def final_upload_confirm(self, results, pbar, final_dict): # Called to confirm upload to live_load_recipe in class DeltaDTB
        """answers[0]: User Action (any key)"""
        if results[0].lower() == 'y':
            self.ctrl_method = 3 # Save PID Program Control Mode (Address 1005H = 3)
            self.furnace.live_load_recipe(final_dict) # Upload the whole dictionary
            pbar.write("[SUCCESS] Recipe uploaded.")
            # Disable tags
            self.in_operation_mode, self.in_cooling_mode, self.check_cool = False, False, False
        else:
            pbar.write("[CANCEL] Upload aborted by user.")
    
    def callback_safe_inject(self, answers, pbar): # Called after finish input for safe_inject
        """
        answers[0]: Step content [°C, min; °C, min] convert to [(°C, min)] Tuple in List
        answers[1]: Selected Pattern index 0 ~ 7
        answers[2]: Selected Step index 0 ~ 7
        """
        try:
            step_content = []
            for s in answers[0].strip(';').split(';'):
                t, m = s.split(',')
                step_content.append((int(t), int(m))) # append(tuple) for inner (), Round to integer
            
            ptn, step = int(answers[1]), int(answers[2])
            
            self.safe_inject(step_content, ptn, step)
            pbar.write(f"[SUCCESS] Injected {len(step_content)} steps into Pattern {ptn} Step {step}")
            # Ignore Tag
        except Exception as e:
            pbar.write(f"[ERROR] Safe Inject failed: {e}")
    
    def callback_at(self, answers, pbar): # Called after finish input for Auto Tune
        """answers[0]: Set Value (°C in integer)"""
        try:
            temp = int(answers[0]) # Round to integer
            if abs(temp - self.current_temp) > self.cfg.auto_tune_safety:
                pbar.write(f"\n[!] SAFETY ERROR: Setpoint ({temp}°C) is too far from Current Temperature ({self.current_temp:.1f}°C).")
                pbar.write(f"AT requires a Setpoint within {self.cfg.auto_tune_safety}°C for safety.")
                # RE-ASK: Trigger the prompt again
                self.setup_input("[RETRY] Enter Target Temp for AT: ", self.callback_at, pbar)
                return 
            # --- Execution ---
            # Ensure the controller is in PID mode (1005H = 0) in order for Auto Tune Process to run
            self.furnace.safe_write(self.furnace.CTRL_METHOD, 0)
            time.sleep(0.5)
            # Set Manual Setpoint (1001H)
            self.furnace.safe_write(self.furnace.SV, int(temp * 10))
            time.sleep(0.5)
            # Start Auto Tune Process and Wait until it done
            self.furnace.start_auto_tune()
            pbar.write(f"[ACTION] Auto-Tune started at {temp}°C")
            # Disable tags
            self.in_cooling_mode, self.check_cool = False, False # Reserve in_operation_mode
            return True # Closes the input buffer
        except (ValueError, IndexError):
            pbar.write("[ERROR] Invalid number! Please enter a temperature(°C).")
            self.setup_input("[RETRY] Enter Target Temp for AT: ", self.callback_at, pbar)
            return False # Keeps input buffer open for retry
        
    def update_plot(self):
        """Live Plot of Temperature vs Time"""
        current_time = time.time()
        if current_time - self.last_plot_update < 5.0: # Only update every 5 seconds to prevent lag
            return
        self.line.set_data(self.time_history, self.temp_history)
        self.ax.relim()
        self.ax.autoscale_view()
        plt.pause(0.001) # Very short pause
        self.last_plot_update = current_time
    
    def save_to_csv(self, writer, f):
        """Standardized output and void data if irrelevant"""
        writer.writerow([ # Void irrelevant data in capture_telemtry
            self.current_time, 
            self.current_temp,
            self.current_rate,
            self.current_power,
            self.kp,
            self.ki,
            self.kd,
            self.current_mode,
            self.next_temp if self.next_temp is not None else "", # Does not autocorrect from path finding logic
            self.ptn if self.ptn is not None else "",
            self.step if self.step is not None else "",
            self.sec_rem if self.sec_rem is not None else "",
            f"{self.coil_temp:.1f}" if self.coil_temp is not None else "" # Coil temp: Blank if not in RAMP
        ])
        f.flush()
    
    def save_recipe_to_disk(self, recipe, filename="PID_history.json"):
        """Saves the current PID program dictionary to a JSON file."""
        try:
            # Create a 'logs' folder if it doesn't exist
            if not os.path.exists('recipes'):
                os.makedirs('recipes')
                
            filepath = os.path.join('recipes', filename)
            with open(filepath, 'w') as f:
                # indent=4 makes the file easy for you to read in Notepad
                json.dump(recipe, f, indent=4)
            print(f"Success: Ramp history saved to {filepath}")
        except Exception as e:
            print(f"Error saving recipe: {e}")

# Initialization
if __name__ == "__main__":
    # Setup Config
    cfg = SystemConfig(port='COM5', voltage=120)
    
    # Setup hardware and heating coil prediction model
    furnace = DeltaDTB(cfg)
    furnace.stop() # Stop the Program immediately
    predictor = HeatingCoilModel(cfg)
    
    # Setup Manager
    manager = PIDProgramManager(furnace, predictor, cfg, target_temp=300)
    
    # 4. RUN
    manager.run_main_loop()
