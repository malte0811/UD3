from math import asin, pi

# Shift s.t. the first harmonic of the output voltages matches the specified voltage
def shift_for_equiv_voltage(effective_voltage: float):
    return 2 / pi * asin(effective_voltage)

# Shift s.t. the first harmonic of the output power (into a series RC circuit) matches that produced by the specified
# voltage
# TODO not sure yet whether the concept makes much sense
def shift_for_equiv_power(effective_voltage: float):
    return 2 / pi * asin(effective_voltage**2)

full_range = 255

def print_lut(name: str, correction_factor):
    values_float = [correction_factor(i / full_range) for i in range(full_range + 1)]
    values_int = [round(full_range * f) for f in values_float]
    assert(values_int[0] == 0)
    assert(values_int[-1] == full_range)
    values_str = ', '.join(map(str, values_int))
    print(f'static uint8_t {name}[{full_range + 1}] = {{{values_str}}};')

print_lut('equiv_voltage', shift_for_equiv_voltage)
print_lut('equiv_power', shift_for_equiv_power)
