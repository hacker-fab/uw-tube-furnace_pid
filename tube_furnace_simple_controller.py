#!/usr/bin/env python3

from tube_furnace_controller import DeltaDTB, SystemConfig
from time import sleep
from argparse import ArgumentParser

previous_temp = 0
loop_delay = 3 #in seconds
current_control_method = 0
moderating_slew = False
slow_heating = False
slow_cooling = False
slew_safety_margin = 0.70
slew_release_margin = 0.65
cooling_safety_margin = 0.65
cooling_release_margin = 0.64
original_target = 25

def set_target(furnace, target):
    furnace.safe_write(furnace.SV, int(target*10))

def set_ctrl_method(furnace, method):
    global current_control_method
    furnace.safe_write(furnace.CTRL_METHOD, method)
    current_control_method = method

def setup(furnace, cfg, target):
    global previous_temp, original_target
    if (target > cfg.temp_limit_high) or (target < cfg.temp_limit_low):
        print("invalid target temp!")
        exit()
    set_target(furnace, target)
    original_target = target
    set_ctrl_method(furnace, cfg.ctrl_method) #0: PID mode
    #get one temp reading to start so slew rate is always valid
    previous_temp = furnace.get_pv()

def calc_slew(prev_temp, curr_temp, timestep):
    slew = ((curr_temp-prev_temp)/timestep)*60 #want to measure in deg C/min
    return round(slew, 2)

def moderate_slew(furnace, cfg, slew, current_temp, target_temp):
    global current_control_method, slow_heating, slow_cooling, moderating_slew
    if ((slew > 0) or slow_heating and not slow_cooling): #we are heating
        if (slew > (cfg.max_allowed_rate*slew_safety_margin)): #kill the heating output
            if not slow_heating:
                print("max heating rate approaching, kill output")
                # set_ctrl_method(furnace, 2) #switch to manual PID mode (so we can override the output duty)
                # furnace.run()
                # furnace.safe_write(furnace.OUT1_VOL, 0)
                furnace.stop()
                slow_heating = True
        else: #return control back to normal
            if (slew < (cfg.max_allowed_rate*slew_release_margin)) and slow_heating:
                # if (current_control_method != 0):
                #     set_ctrl_method(furnace, cfg.ctrl_method)
                furnace.run()
                set_target(furnace, original_target)
                slow_heating = False
    elif ((slew <= 0) or slow_cooling): #we are cooling
        if (abs(slew) > (cfg.max_allowed_rate*slew_safety_margin)): #turn on the heating output @50% duty
            if not slow_cooling:
                print("max cooling rate approaching, heating at 100% duty")
                # set_ctrl_method(furnace, 2) #switch to manual PID mode (so we can override the output duty)
                set_target(furnace, current_temp + 50)
                # furnace.safe_write(furnace.OUT1_VOL, 1000) #TODO maybe ramp duty iteratively until it stabilizes
                slow_cooling = True
        else: #return control back to normal
            if (abs(slew) > (cfg.max_allowed_rate*cooling_release_margin)) and slow_cooling:
                # if (current_control_method != 0):
                #     set_ctrl_method(furnace, cfg.ctrl_method)
                set_target(furnace, original_target)
                slow_cooling = False

def main_loop(furnace, cfg):
    global previous_temp, loop_delay
    target_temp = furnace.get_sv()
    try:
        while True:
            current_temp = furnace.get_pv()
            # target_temp = furnace.get_sv()
            current_duty = furnace.get_power_percent()
            slew = calc_slew(previous_temp, current_temp, loop_delay)
            print(f"temp: {current_temp}, slew rate: {slew} target: {target_temp}, duty: {current_duty}")
            moderate_slew(furnace, cfg, slew, current_temp, target_temp)
            previous_temp = current_temp
            sleep(loop_delay)
    except KeyboardInterrupt:
        print("ctrl+c detected, shutdown...")
        furnace.stop()
        exit()

if __name__ == "__main__":
    parser = ArgumentParser(
        prog='tube_furnace_simple_controller.py',
        description='simplified control for the DeltaDTB PID controller'
    )
    parser.add_argument('-p', '--port', required=True)
    parser.add_argument('-t', '--target_temp', type=float)
    parser.add_argument('-i', '--initialize', action='store_true')
    parser.add_argument('-m', '--control_method', choices=['PID', 'ONOFF'], default='ONOFF')
    args = parser.parse_args()

    if (args.control_method == 'PID'):
        ctrl_method = 0
    elif (args.control_method == 'ONOFF'):
        ctrl_method = 1
    else: #default
        ctrl_method = 1

    cfg = SystemConfig(port=args.port, voltage=120, ctrl_method=ctrl_method)

    furnace = DeltaDTB(cfg)
    if not furnace.stop():
        print("failed to communicate with furnace")
        furnace.ser.close()
        exit()

    #TMP hack override the P value
    furnace.safe_write(furnace.PID_P, 100)
    furnace.safe_write(furnace.PID_I, 150)
    #TMP HACK read out the PID values on the fly
    P_val = furnace.parse_response(furnace.read_register(furnace.PID_P))
    I_val = furnace.parse_response(furnace.read_register(furnace.PID_I))
    D_val = furnace.parse_response(furnace.read_register(furnace.PID_D))
    print(f'P: {P_val}, I: {I_val}, D:{D_val}')


    #just initialize and exit
    if args.initialize:
        furnace.initialize_hardware()
        exit()
    elif args.target_temp is None:
        parser.error("target temp not defined!")
    else:
        setup(furnace, cfg, args.target_temp)
        sleep(loop_delay)
        furnace.run()
        main_loop(furnace, cfg)
