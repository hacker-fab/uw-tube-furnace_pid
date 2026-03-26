#!/usr/bin/env python3

from tube_furnace_controller import DeltaDTB, SystemConfig
from time import sleep
from argparse import ArgumentParser

previous_temp = 0
loop_delay = 3 #in seconds
current_control_method = 0
moderating_slew = False

def set_target(furnace, target):
    furnace.safe_write(furnace.SV, int(target*10))

def set_ctrl_method(furnace, method):
    global current_control_method
    furnace.safe_write(furnace.CTRL_METHOD, method)
    current_control_method = method

def setup(furnace, cfg, target):
    global previous_temp
    if (target > cfg.temp_limit_high) or (target < cfg.temp_limit_low):
        print("invalid target temp!")
        exit()
    set_target(furnace, target)
    set_ctrl_method(furnace, 0) #0: PID mode
    #get one temp reading to start so slew rate is always valid
    previous_temp = furnace.get_pv()

def calc_slew(prev_temp, curr_temp, timestep):
    return (curr_temp-prev_temp)/timestep

def moderate_slew(furnace, cfg, slew, current_temp, target_temp):
    global current_control_method, moderating_slew
    if (current_temp < target_temp): #we are heating
        if (abs(slew) > cfg.max_allowed_rate): #kill the heating output
            if not moderating_slew:
                print("max heating rate approaching, kill output")
                set_ctrl_method(furnace, 2) #switch to manual PID mode (so we can override the output duty)
                furnace.safe_write(furnace.OUT1_VOL, 0)
                moderating_slew = True
        else: #return control back to normal
            if (current_control_method != 0):
                set_ctrl_method(furnace, 0)
            moderating_slew = False
    elif (current_temp > target_temp): #we are cooling
        if (abs(slew) > cfg.max_allowed_rate): #turn on the heating output @50% duty
            if not moderating_slew:
                print("max cooling rate approaching, heating at 50% duty")
                set_ctrl_method(furnace, 2) #switch to manual PID mode (so we can override the output duty)
                furnace.safe_write(furnace.OUT1_VOL, 50) #TODO maybe ramp duty iteratively until it stabilizes
                moderating_slew = True
        else: #return control back to normal
            if (current_control_method != 0):
                set_ctrl_method(furnace, 0)
            moderating_slew = False

def main_loop(furnace, cfg):
    global previous_temp, loop_delay
    try:
        while True:
            current_temp = furnace.get_pv()
            target_temp = furnace.get_sv()
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
    args = parser.parse_args()

    cfg = SystemConfig(port=args.port, voltage=120)

    furnace = DeltaDTB(cfg)
    furnace.stop()

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
