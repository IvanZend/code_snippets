
/*
The code snippets presented in this document are simplified, anonymized representations based on my experience in commercial embedded systems projects.

They are provided solely for demonstration purposes and do not contain proprietary or confidential information.
*/


/*

1. EMG Signal Filtering and Threshold Logic

-------------------------------------------

Developed an adaptive filtering method for real-time EMG signal classification.
Implemented threshold logic to detect gesture activation based on filtered ADC values.
Function names and variables generalized from production firmware.

*/

// update moving average variable
void Emg::update_average_filter(uint32_t adc_val)
{
    uint32_t filter_deduction = average_filter / NMBR_OF_SUMMED_SAMPLES;
    average_filter = (average_filter - filter_deduction) + adc_val;
}

// check if a new ADC value differs enough from a previous one
bool Emg::emg_threshold_passed(uint8_t delta_percent)
{
    uint32_t filter_pass_val = filter_pass_threshold(adc_dma_buf[init.dma_indx], delta_percent);

    if (emg_logic_high)
    {
        if (adc_dma_buf[init.dma_indx] <= filter_pass_val)
        {
            emg_logic_high = false;
        }
    }
    else
    {
        if (adc_dma_buf[init.dma_indx] > filter_pass_val)
        {
            emg_logic_high = true;
        }
    }

    return emg_logic_high;
}

// calculate threshold based on a percentage deviation from average
uint16_t Emg::filter_pass_threshold(uint32_t adc_val, uint8_t threshold_precent)
{
    uint32_t average_val = average_filter / NMBR_OF_SUMMED_SAMPLES;   // get a single ADC value from a moving average
    return (average_val + ((average_val * threshold_precent) / 100));
}


/*

2. Motor Control via PWM and GPIO

---------------------------------

Designed start/stop routines for DC motor with current sensing and direction control.

*/

void Motor::forward()
{
    if (!moving_now)
    {
        HAL_GPIO_WritePin(init.phase.port, init.phase.pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(init.sleep.port, init.sleep.pin, GPIO_PIN_SET);
        HAL_TIM_PWM_Start(init.pwm.timer, init.pwm.channel);
        moving_now = true;
    }
    timer = 0;
}


/*

3. Buffered BLE Data Transmission

---------------------------------

Implemented a buffer-based data transmission routine with timestamped gesture and sensor data.

*/

// transfer sampled data from the main ring buffer to a BLE buffer formatted according to a custom protocol
void Board::copy_sample_from_memory_to_ble_buf(uint16_t ble_buf_indx, uint32_t memory_indx)
{
    uint16_t ble_byte_indx = ble_buf_indx * BLE_FRAME_BYTES;
    ble_buf[STRING_HANDLER_BYTES + (ble_byte_indx)] = frames_ring_buf.buf_array[memory_indx].timestamp>>ONE_BYTE_SHIFT;   // split 16-bit timestamp into two 8-bit bytes
    ble_buf[STRING_HANDLER_BYTES + (ble_byte_indx + 1)] = frames_ring_buf.buf_array[memory_indx].timestamp;               // 
    ble_buf[STRING_HANDLER_BYTES + (ble_byte_indx + 2)] = frames_ring_buf.buf_array[memory_indx].gesture;
    ble_buf[STRING_HANDLER_BYTES + (ble_byte_indx + 3)] = frames_ring_buf.buf_array[memory_indx].motor_current;
    ble_buf[STRING_HANDLER_BYTES + (ble_byte_indx + 4)] = frames_ring_buf.buf_array[memory_indx].battery_voltage;
}
