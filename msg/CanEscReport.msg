# can_esc_report.msg
uint64 timestamp      # time since system start (microseconds)

# esc report 1
float32 voltage_in
float32 current_in
float32 current_out
float32 freq_out
uint16 rpm

# esc report 2
uint16 throttle_desired
bool pwm_channel
bool can1_channel
bool can2_channel
bool uart_channel
uint16 throttle_current
int16 t_mcu
int16 t_module

# esc report 3
uint32 status_flags
uint16 theta
int16 t_motor

