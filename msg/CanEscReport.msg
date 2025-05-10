# can_esc_report.msg
uint64 timestamp      # time since system start (microseconds)

int8 esc_index

float32 voltage_in
float32 current_in
float32 current_out
int16 t_mcu
int16 t_module
int16 t_cap            # capacitor temperature
int16 t_mos
int16 t_motor

bool pwm_channel
bool can1_channel
bool can2_channel
bool uart_channel

uint16 recv_pwm
uint16 comm_pwm
uint16 rpm

uint8 v_modulation
uint32 status_flags

