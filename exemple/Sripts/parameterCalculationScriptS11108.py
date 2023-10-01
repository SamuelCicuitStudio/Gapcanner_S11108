def format_time(value, unit):
    if unit == "Hz":
        if value >= 1e6:
            unit = "MHz"
            formatted_value = value / 1e6
        elif value >= 1e3:
            unit = "kHz"
            formatted_value = value / 1e3
        else:
            formatted_value = value
    else:
        if value >= 1000:
            unit = "ms"
            formatted_value = value / 1000
        elif value >= 0.001:
            unit = "us"
            formatted_value = value * 1000
        elif value >= 1e-6:
            unit = "ns"
            formatted_value = value * 1e6
        else:
            unit = "ps"
            formatted_value = value * 1e9

    # Round to two decimal places if the value is not zero
    if formatted_value != 0:
        formatted_value = round(formatted_value, 2)

    return formatted_value, unit



def calculate_ST_frequency_and_duty_cycle(master_clock_frequency_Hz):
    # Calculate tpi(ST) in microseconds
    tpi_ms = (2140 / master_clock_frequency_Hz) *1000

    # Calculate tlp(ST) in milliseconds
    thp_ms = (2140 / master_clock_frequency_Hz - 92 / master_clock_frequency_Hz )*1000
    
    # Calculate thp(ST) in milliseconds
    tlp_ms = tpi_ms - thp_ms
      
    # Calculate ST frequency in Hertz
    st_frequency_Hz = (1 / tpi_ms)*1000
    
    # Calculate duty cycle of ST signal
    duty_cycle_ST = (thp_ms / tpi_ms) * 100
    
    # Calculate thp(Save) in milliseconds
    thp_save = thp_ms + 88 * 1000/master_clock_frequency_Hz
    
    # Calculate tlp(Save) in milliseconds
    tlp_save = tpi_ms - thp_save
    
    # Calculate duty cycle of Save signal
    duty_cycle_Save = (thp_save / (thp_save+tlp_save)) * 100
    
    # Format times to adjust the printing caliber
    st_frequency_value, st_frequency_unit = format_time(st_frequency_Hz, "Hz")
    tlp_value, tlp_unit = format_time(tlp_ms, "ms")
    thp_value, thp_unit = format_time(thp_ms, "ms")
    tpi_value, tpi_unit = format_time(tpi_ms, "ms")
    tlp_save_value, tlp_save_unit = format_time(tlp_save, "ms")
    thp_save_value, thp_save_unit = format_time(thp_save, "ms")

    return (
        st_frequency_value, st_frequency_unit, 
        duty_cycle_ST,
        duty_cycle_Save, 
        tlp_value, tlp_unit, 
        thp_value, thp_unit, 
        tpi_value, tpi_unit,
        tlp_save_value,tlp_save_unit,
        thp_save_value,thp_save_unit,
    )

# Input the master clock frequency in Hertz
master_clock_frequency_Hz = float(input("Enter the master clock frequency (Hz): "))

# Calculate ST frequency, duty cycle, and related values
(
    st_frequency_value, st_frequency_unit, 
    duty_cycle_ST,
    duty_cycle_Save, 
    tlp_value, tlp_unit, 
    thp_value, thp_unit, 
    tpi_value, tpi_unit,
    tlp_save_value,tlp_save_unit,
    thp_save_value,thp_save_unit,
) = calculate_ST_frequency_and_duty_cycle(master_clock_frequency_Hz)

# Print the results with the appropriate units and two decimal places if not zero
print(f"ST frequency = Save frequency: {st_frequency_value:.2f} {st_frequency_unit}")
print(f"Duty cycle of ST signal: {duty_cycle_ST:.2f}%")
print(f"Duty cycle of Save signal: {duty_cycle_Save:.2f}%")
print(f"tlp(ST): {tlp_value:.2f} {tlp_unit}")
print(f"thp(ST): {thp_value:.2f} {thp_unit}")
print(f"tpi(ST): {tpi_value:.2f} {tpi_unit}")
print(f"tlp(Save): {tlp_value:.2f} {tlp_unit}")
print(f"thp(Save): {thp_value:.2f} {thp_unit}")
