# generates temperature conversion code in therm.c
# usage: awk -f omega.awk omega_negative.txt omega_positive.txt
# data are copied from www.omgea.com/temperature/Z/pdf/z204-206.pdf

# read reference thermocouple data
# assumes it is in sorted order
NF == 13 {
    ref_temp = $1
    for (i=2; i <= 11; i++) {
        # the negative file
        if ( NR == FNR )
            this_temp = ref_temp - 12 + i
        # the positive file
        else
            this_temp = ref_temp + i - 2

        # store voltage data
        j++
        temperature[j] = this_temp
        voltage[j] = $i
    }
}

# print out C code
END {
    n_temperatures = j

    # iteration assumes that temperatures are in ascending order
    temp_interval = 10
    for (i=1; i < n_temperatures - temp_interval; i += temp_interval) {
        # consider piecewise intervals
        min_temp = temperature[i]
        max_temp = temperature[i+temp_interval]
        min_voltage = voltage[i]
        max_voltage = voltage[i+temp_interval]

        # for debugging only
        #min_temp=5
        #max_temp=10
        #min_voltage=0.198
        #max_voltage=0.397

        # convert to hex
        c1 = hex_string(min_voltage)
        c2 = hex_string(max_voltage)
        sp = max_temp-min_temp
        d1 = hex_string(max_voltage-min_voltage)
        i1 = min_temp ".0f"
        
        # generate C code
        printf "\t"
        if ( i > 1)
            printf "else "
        printf "if (code > %s && code < %s) temp = (float)(%s*(temp-%s)) / %s", c1, c2, sp, c1, d1
        if ( min_temp >= 0 )
            printf " +"
        printf " %s;  // %4d C to %4d C\n", i1, min_temp, max_temp
        
        # for debugging only
        #break
    }
}

function hex_string(hex,   str) {
    str = sprintf("%04X", hex*1000.0/7.8125)
    str = "0x" substr(str,length(str)-3,length(str))
    return str
}
