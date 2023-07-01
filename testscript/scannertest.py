# SCAN2000 test script
# via SCPI
# Target: DMM6500 with SCAN2000-20, with shunts on channels 1 and 11
# Additional need: serial console on the serial debug output of the SCAN2000 card.
#
# The script will test channels 1, 11 in a loop.
# We can detect which channel is really selected by looking at the 2W resistance:
# Due to the SSR internal resistance, CH1 should read about 2 Ohm, and CH11 about 4 Ohm
# If by error another channel was activated, it should read "Overflow"
#
# The script stop testing once an error is detected.
# One should use it in conjunction with a serial console on serial debug output of the SCAN2000 card.
# That will help see communication/interpretation errors.
# As the test script stop after an error, it suffices to look at the latest messages (ignoring the "Idle" messages).

import pyvisa as visa
import time

dev_target = None

DEBUG = False

# SCPI Addresses:
# Target
ADDR_TARGET = "TCPIP::192.168.7.205::INSTR"
NPLC_MAX_TARGET = 10

# make sure you use valid values, for all devices.
MEASUREMENT_NPLC = 1

TARGET_CH1 = 2
TARGET_CH11 = 4
TARGET_DEVIANCE_ABS = 1


def inst_target_init(rm):
    global inst_target
    inst_target = rm.open_resource(ADDR_TARGET)

    nplc = MEASUREMENT_NPLC
    avg_filter = 1
    if nplc > NPLC_MAX_TARGET:
        nplc = NPLC_MAX_TARGET
        avg_filter = MEASUREMENT_NPLC / NPLC_MAX_TARGET
        # in ms
        inst_target.timeout = 10000

    inst_target.write("*CLS")

    # check ID
    s = inst_target.query("*IDN?").strip()
    if "DMM6500" not in s:
        print(f'ERROR: device ID is unexpected: "{s}"')
        return False

    # set to voltage measurement, inputs 1 and 11
    inst_target.write("SENS:FUNC 'RES', (@1,11)")
    inst_target.write(f"SENS:RES:NPLC {nplc}, (@1,11)")
    inst_target.write("RES:RANG 100, (@1,11)")
    inst_target.write("RES:LINE:SYNC 1, (@1,11)")
    if avg_filter <= 1:
        inst_target.write("RES:AVER 0, (@1,11)")
    else:
        inst_target.write(f"RES:AVER:COUNT {avg_filter}, (@1,11)")
        inst_target.write("RES:AVER:TCON REP, (@1,11)")
        inst_target.write("RES:AVER:STAT 1, (@1,11)")

    s = inst_target.query("SYST:ERR?").strip()
    if not s.startswith("0,\"No error"):
        print(f'ERROR during init: "{s}"')
        return False
    return True


def getTargetCh(ch):
    global inst_target

    inst_target.write("ABOR")
    inst_target.write("ROUT:OPEN:ALL")
    inst_target.write(f"ROUT:CLOS (@{ch})")
    try:
        # TODO: This sometimes throws an I/O error. Debug further.
        s = inst_target.query('READ? "defbuffer1", READ, CHAN, STAT').strip()
    except visa.VisaIOError as e:
        # Handle the exception.
        print(f"ERROR: {e.description}")
        s = ""
        
    # inst_target.write(f"ROUT:OPEN (@{ch})")
    # inst_target.write("ROUT:OPEN:ALL")

    l = s.split(",")
    if len(l) != 3:
        print(f'ERROR reading from channel {ch}, reply = "{s}"')
        return None
    try:
        if int(l[1]) != int(ch):
            print(f"ERROR reading from channel {ch}, got reply from channel {l[1]}")
            return None
        if int(l[2]) != 0:
            print(f"ERROR reading from channel {ch}, got status code {l[2]}")
            return None
    except:
        print(f'ERROR reading from channel {ch}, reply = "{s}"')
        return None

    f = float(l[0])
    return f


def format_float(val):
    return f"{val:+.2f}".replace(".", ",")


def readDevices():
    global inst_target

    print(f"Using NPLC {NPLC_MAX_TARGET}")

    rm = visa.ResourceManager()
    if DEBUG:
        print(rm.list_resources())

    print("Opening target.")
    if not inst_target_init(rm):
        return 1

    print("Init OK")

    err = False
    nr = 1
    while not err:
        t = TARGET_CH1
        ch = 1
        f = getTargetCh(ch)
        if f is None:
            err = True
            continue
        if abs(f - t) > TARGET_DEVIANCE_ABS:
            print(f"ERROR: CH{ch} expected {f} ohm, got {t} ohm. Channel error.")
            err = True
            continue
        ch1 = f

        t = TARGET_CH11
        ch = 11
        f = getTargetCh(ch)
        if f is None:
            err = True
            continue
        if abs(f - t) > TARGET_DEVIANCE_ABS:
            print(f"ERROR: CH{ch} expected {f} ohm, got {t} ohm. Channel error.")
            err = True
            continue
        ch11 = f

        print(f"{nr};OK;{format_float(ch1)};{format_float(ch11)}")
        nr += 1

    print("Halted")


if __name__ == "__main__":
    # visa.log_to_screen()
    readDevices()
