
/**
* Musuko blocks
*/

// GPIO -------------------------------------------------
enum GPIO_port {
    //% block="P0"
    P0 = 0,
    //% block="P1"
    P1 = 1,
    //% block="P2"
    P2 = 2,
    //% block="P3"
    P3 = 3,
    //% block="P4"
    P4 = 4,
    //% block="P5"
    P5 = 5
};

enum GPIO_State {
    //% block="LOW"
    LOW = 0,
    //% block="HIGH"
    HIGH = 1
};

enum LED_State {
    //% block="ON"
    ON = 0,
    //% block="OFF"
    OFF = 1
};

// UART -------------------------------------------------
enum UART_Ch {
    //% block="A"
    UARTA = 0,
    //% block="B"
    UARTB = 1
};

enum UART_Baud {
    //% block="2400"
    U2400 = 2400,
    //% block="4800"
    U4800 = 4800,
    //% block="9600"
    U9600 = 9600,
    //% block="19200"
    U19200 = 19200,
    //% block="38400"
    U38400 = 38400,
    //% block="57600"
    U57600 = 57600,
    //% block="115200"
    U115200 = 115200,
    //% block="230400"
    U230400 = 230400,
    //% block="460800"
    U460800 = 460800,
    //% block="921600"
    U921600 = 921600,
};

// Motor ------------------------------------------------
enum Motor_Ch {
    //% block="M1"
    M1 = 0,
    //% block="M2"
    M2 = 1,
    //% block="M3"
    M3 = 2,
    //% block="M4"
    M4 = 3
};

enum Motor_Dir {
    //% block="CW"
    CW = 0,
    //% block="CCW"
    CCW = 1
};

enum Wheel_Dir {
    //% block="Back"
    Back = 0,
    //% block="Front"
    Front = 1
};

enum Robot_Dir {
    //% block="Back"
    Back = 0,
    //% block="Front"
    Front = 1,
    //% block="Left"
    Left = 2,
    //% block="Right"
    Right = 3,
    //% block="Rotate cw"
    CW = 4,
    //% block="Rotate ccw"
    CCW = 5
};

// Servo ------------------------------------------------
enum Servo_Ch {
    //% block="Servo0"
    S0 = 0,
    //% block="Servo1"
    S1 = 1,
    //% block="Servo2"
    S2 = 2,
    //% block="Servo3"
    S3 = 3
};

// MPU6050  ---------------------------------------------
// Enumeration of Axis (X, Y & Z)
enum Axis_XYZ {
    //% block="X"
    x = 0,
    //% block="Y"
    y = 1,
    //% block="Z"
    z = 2
};

// Sensitivity of Accelerometer
enum AccelSen {
    //% block="2g"
    range_2_g,
    //% block="4g"
    range_4_g,
    //% block="8g"
    range_8_g,
    //% block="16g"
    range_16_g
};

// Sensitivity of Gyroscope
enum GyroSen {
    //% block="250dps"
    range_250_dps,
    //% block="500dps"
    range_500_dps,
    //% block="1000dps"
    range_1000_dps,
    //% block="2000dps"
    range_2000_dps
};

//--------------------------------------------------------------------------
//% weight=11 color=#0000ff icon="\uf085" block="MUSUKO:BIT"
//% groups=['Headlights', 'DC Motors', 'Maker Line', 'Ultrasonic']
namespace Musuko {

    //----------------------------------------------------------------------
    // GPS
    //----------------------------------------------------------------------
    let gls_lat = 0.0;
    let gls_lng = 0.0;

    /**
     * Call gps_setup on start
     */
    //% group="GPS Receiver"
    //% weight=11
    //% blockGap=8
    //% block="Initialize GPS"
    export function gps_setup(): void {
        serial.redirect(
            SerialPin.P0,
            SerialPin.P1,
            BaudRate.BaudRate38400
        );
        serial.setRxBufferSize(255);
    }

    /**
     * Call gps_update function in forever loop to update GPS data
     */
    //% group="GPS Receiver"
    //% weight=10
    //% blockGap=8
    //% block="Update GPS data"
    export function gps_update(): void {
        let msg = serial.readBuffer(0).toString();

        if (msg.length > 60) {
            const myArray = msg.split(",");

            if (myArray[0] === "$GNGGA") {
                gls_lat = parseFloat(myArray[2]) / 100.0;
                gls_lng = parseFloat(myArray[4]) / 100.0;
            }
        }
    }

    /**
     * Get gps latitude degree
     */
    //% group="GPS Receiver"
    //% weight=9
    //% blockGap=8
    //% block="Read GPS latitude"
    export function gps_lat(): number {
        return gls_lat;
    }

    /**
     * Get gps longitude in degree
     */
    //% group="GPS Receiver"
    //% weight=8
    //% blockGap=40
    //% block="Read GPS longitude"
    export function gps_lng(): number {
        return gls_lng;
    }

    //----------------------------------------------------------------------
    // Robot
    //----------------------------------------------------------------------
    /**
    * Set robot move
    */
    //% group="Omni robot"
    //% weight=73
    //% blockGap=8
    //% block="Robot run to %dir speed %speed"
    export function robot_run(dir: Robot_Dir, speed: number): void {
        switch (dir) {
            case Robot_Dir.Back:
                omni_run(Wheel_Dir.Back, speed, Wheel_Dir.Back, speed, Wheel_Dir.Back, speed, Wheel_Dir.Back, speed);
                break;
            case Robot_Dir.Front:
                omni_run(Wheel_Dir.Front, speed, Wheel_Dir.Front, speed, Wheel_Dir.Front, speed, Wheel_Dir.Front, speed);
                break;
            case Robot_Dir.Left:
                omni_run(Wheel_Dir.Back, speed, Wheel_Dir.Front, speed, Wheel_Dir.Back, speed, Wheel_Dir.Front, speed);
                break;
            case Robot_Dir.Right:
                omni_run(Wheel_Dir.Front, speed, Wheel_Dir.Back, speed, Wheel_Dir.Front, speed, Wheel_Dir.Back, speed);
                break;
            case Robot_Dir.CW:
                omni_run(Wheel_Dir.Back, speed, Wheel_Dir.Back, speed, Wheel_Dir.Front, speed, Wheel_Dir.Front, speed);
                break;
            case Robot_Dir.CCW:
                omni_run(Wheel_Dir.Front, speed, Wheel_Dir.Front, speed, Wheel_Dir.Back, speed, Wheel_Dir.Back, speed);
                break;
        }
    }

    /**
    * Set robot wheel
    */
    //% group="Omni robot"
    //% weight=72
    //% blockGap=8
    //% block="Set wheel]n wheel1 to %dir1 %speed1\n wheel2 to %dir2 %speed2\n wheel3 to %dir3 %speed3\n wheel4 to %dir4 %speed4"
    export function omni_run(dir1: Wheel_Dir, speed1: number, dir2: Wheel_Dir, speed2: number, dir3: Wheel_Dir, speed3: number, dir4: Wheel_Dir, speed4: number): void {
        if (dir1 == Wheel_Dir.Front) { motor_run(Motor_Ch.M1, Motor_Dir.CCW, speed1); }
        else { motor_run(Motor_Ch.M1, Motor_Dir.CW, speed1) }

        if (dir2 == Wheel_Dir.Front) { motor_run(Motor_Ch.M2, Motor_Dir.CCW, speed2); }
        else { motor_run(Motor_Ch.M2, Motor_Dir.CW, speed2) }

        if (dir3 == Wheel_Dir.Front) { motor_run(Motor_Ch.M3, Motor_Dir.CCW, speed3); }
        else { motor_run(Motor_Ch.M3, Motor_Dir.CW, speed3) }

        if (dir4 == Wheel_Dir.Front) { motor_run(Motor_Ch.M4, Motor_Dir.CCW, speed4); }
        else { motor_run(Motor_Ch.M4, Motor_Dir.CW, speed4) }
    }

    /**
     * Set wheel rotation
     */
    //% group="Omni robot"
    //% weight=71
    //% blockGap=8
    //% block="Set wheel %ch run to %dir speed %speed"
    export function wheel_run(ch: Motor_Ch, dir: Wheel_Dir, speed: number): void {
        if (dir == Wheel_Dir.Front) { motor_run(ch, Motor_Dir.CCW, speed); }
        else { motor_run(ch, Motor_Dir.CW, speed); }
    }

    /**
     * Stop wheel
     */
    //% group="Omni robot"
    //% weight=70
    //% blockGap=40
    //% block="Stop all wheel"
    export function stop_all_wheel(): void {
        PCA9685_setDutyCycle(8, 0)
        PCA9685_setDutyCycle(9, 0)
        PCA9685_setDutyCycle(10, 0)
        PCA9685_setDutyCycle(11, 0)
        PCA9685_setDutyCycle(12, 0)
        PCA9685_setDutyCycle(13, 0)
        PCA9685_setDutyCycle(14, 0)
        PCA9685_setDutyCycle(15, 0)
    }

    //----------------------------------------------------------------------
    // Motor
    //----------------------------------------------------------------------
    /**
     * Set motor speed and direction
     */
    //% group="Motor driver"
    //% weight=40
    //% blockGap=8
    //% block="Set motor %ch run to %dir speed %speed"
    export function motor_run(ch: Motor_Ch, dir: Motor_Dir, speed: number): void {
        let pwm_cw = 0;
        let pwm_ccw = 0;

        if (dir == 0) {
            pwm_cw = speed;
            pwm_ccw = 0;
        }
        else if (dir == 1) {
            pwm_cw = 0;
            pwm_ccw = speed;
        }
        else {
            pwm_cw = 0;
            pwm_ccw = 0;
        }

        if (ch == 0) {
            PCA9685_setDutyCycle(8, pwm_cw)
            PCA9685_setDutyCycle(9, pwm_ccw)
        }
        else if (ch == 1) {
            PCA9685_setDutyCycle(10, pwm_cw)
            PCA9685_setDutyCycle(11, pwm_ccw)
        }
        else if (ch == 2) {
            PCA9685_setDutyCycle(12, pwm_cw)
            PCA9685_setDutyCycle(13, pwm_ccw)
        } else if (ch == 3) {
            PCA9685_setDutyCycle(14, pwm_cw)
            PCA9685_setDutyCycle(15, pwm_ccw)
        }
    }

    /**
     * Stop motor
     */
    //% group="Motor driver"
    //% weight=39
    //% blockGap=8
    //% block="Set motor %ch break"
    export function motor_break(ch: Motor_Ch): void {
        let pwm_cw = 0;
        let pwm_ccw = 0;

        if (ch == 0) {
            PCA9685_setDutyCycle(8, pwm_cw)
            PCA9685_setDutyCycle(9, pwm_ccw)
        }
        else if (ch == 1) {
            PCA9685_setDutyCycle(10, pwm_cw)
            PCA9685_setDutyCycle(11, pwm_ccw)
        }
        else if (ch == 2) {
            PCA9685_setDutyCycle(12, pwm_cw)
            PCA9685_setDutyCycle(13, pwm_ccw)
        } else if (ch == 3) {
            PCA9685_setDutyCycle(14, pwm_cw)
            PCA9685_setDutyCycle(15, pwm_ccw)
        }
    }

    /**
     * Stop motor
     */
    //% group="Motor driver"
    //% weight=38
    //% blockGap=40
    //% block="Stop all motor"
    export function motor_stop_all(): void {
        PCA9685_setDutyCycle(8, 0)
        PCA9685_setDutyCycle(9, 0)
        PCA9685_setDutyCycle(10, 0)
        PCA9685_setDutyCycle(11, 0)
        PCA9685_setDutyCycle(12, 0)
        PCA9685_setDutyCycle(13, 0)
        PCA9685_setDutyCycle(14, 0)
        PCA9685_setDutyCycle(15, 0)
    }

    //----------------------------------------------------------------------
    // SERVO
    //----------------------------------------------------------------------
    /**
     * Set servo angle
     */
    //% group="Servo"
    //% weight=39
    //% blockGap=40
    //% block="Set servo motor %ch to %degrees degree"
    export function set_servo(ch: Servo_Ch, degrees: number): void {
        if (ch == 0) {
            PCA9685_setServo(0, degrees)
        }
        else if (ch == 1) {
            PCA9685_setServo(1, degrees)
        }
        else if (ch == 2) {
            PCA9685_setServo(2, degrees)
        }
        else if (ch == 3) {
            PCA9685_setServo(3, degrees)
        }
    }


    //----------------------------------------------------------------------
    // GPIO
    //----------------------------------------------------------------------
    const PCF8575_ADDR = (0x21);

    /**
    * @param value : port output value
    */
    //% group="GPIO"
    //% weight=20
    //% blockGap=8
    //% block="write port to %value"
    export function io_write(value: number): void {
        pins.i2cWriteNumber(PCF8575_ADDR, value, NumberFormat.UInt16LE, false);
    }

    /**
    * @param pin   : pin number 1-8
    * @param value : port status value
    */
    //% group="GPIO"
    //% weight=19
    //% blockGap=8
    //% block="write pin %pin to %value"
    export function io_write_pin(pin: GPIO_port, value: GPIO_State) {
        let port = pins.i2cReadNumber(PCF8575_ADDR, NumberFormat.UInt16LE, false);

        if (value == GPIO_State.LOW) { port = port & (~(0x01 << pin)); }
        else { port = port | ((0x01 << pin)); }

        io_write(port);
    }

    /**
    * @param pin   : pin number 1-8
    * @param value : port status value
    */
    //% group="GPIO"
    //% weight=18
    //% blockGap=8
    //% block="set LED %pin to %value"
    export function io_write_led(pin: GPIO_port, value: LED_State) {
        if (value == LED_State.ON) { io_write_pin(pin, GPIO_State.LOW); }
        else { io_write_pin(pin, GPIO_State.HIGH);; }
    }



    /**
    * @param value : port status value
    */
    //% group="GPIO"
    //% weight=17
    //% blockGap=8
    //% block="get port value"
    export function io_read(): number {
        let port = pins.i2cReadNumber(PCF8575_ADDR, NumberFormat.UInt16LE, false);
        return port;
    }

    /**
    * @param pin   : pin number 1-8
    * @param value : port status value
    */
    //% group="GPIO"
    //% weight=16
    //% blockGap=40
    //% block="get pin %pin state"
    export function io_read_pin(pin: GPIO_port): boolean {
        let port = pins.i2cReadNumber(PCF8575_ADDR, NumberFormat.UInt16LE, false);
        return ((port >> pin) & 0x01) == 0x01 ? true : false;
    }

    //----------------------------------------------------------------------
    // UART
    //----------------------------------------------------------------------
    /* Device Address */
    /* A:VDD, B:GND, C:SCL, D:SDA */
    const SC16IS750_ADDRESS_BB = (0X9A);

    // General Registers
    const SC16IS750_REG_RHR = (0x00);
    const SC16IS750_REG_THR = (0X00);
    const SC16IS750_REG_IER = (0X01);
    const SC16IS750_REG_FCR = (0X02);
    const SC16IS750_REG_IIR = (0X02);
    const SC16IS750_REG_LCR = (0X03);
    const SC16IS750_REG_MCR = (0X04);
    const SC16IS750_REG_LSR = (0X05);
    const SC16IS750_REG_MSR = (0X06);
    const SC16IS750_REG_SPR = (0X07);
    const SC16IS750_REG_TCR = (0X06);
    const SC16IS750_REG_TLR = (0X07);
    const SC16IS750_REG_TXLVL = (0X08);
    const SC16IS750_REG_RXLVL = (0X09);
    const SC16IS750_REG_IODIR = (0X0A);
    const SC16IS750_REG_IOSTATE = (0X0B);
    const SC16IS750_REG_IOINTENA = (0X0C);
    const SC16IS750_REG_IOCONTROL = (0X0E);
    const SC16IS750_REG_EFCR = (0X0F);
    const SC16IS750_REG_DLL = (0x00);
    const SC16IS750_REG_DLH = (0X01);
    const SC16IS750_REG_EFR = (0X02);
    const SC16IS750_CRYSTCAL_FREQ = (1843200);
    const SC16IS752_CHANNEL_A = 0x00;
    const SC16IS752_CHANNEL_B = 0x01;
    const SC16IS752_CHANNEL_BOTH = 0x00;

    let peek_buf = [- 1, -1];
    let peek_flag = [0, 0];
    let fifo_available = [0, 0];
    let initialized = false;
    let SC16IS752_addr = SC16IS750_ADDRESS_BB >> 1;

    //% group="UART"
    //% weight=40
    //% blockGap=8
    //% block="Setup uart baud %baud_A %baud_B bits/sec"
    export function uart_ext_begin(baud_A: UART_Baud, baud_B: UART_Baud): void {
        uart_ext_Initialize();
        uart_ext_beginA(baud_A);
        uart_ext_beginB(baud_B);
    }

    //% group="UART"
    //% weight=39
    //% blockGap=8
    //% block="Setup uart A %baud_A bits/sec"
    export function uart_ext_beginA(baud_A: UART_Baud): void {
        if (!initialized) { uart_ext_Initialize(); }
        uart_ext_FIFOEnable(SC16IS752_CHANNEL_A, 1);
        uart_ext_SetBaudrate(SC16IS752_CHANNEL_A, baud_A);
        uart_ext_SetLine(SC16IS752_CHANNEL_A, 8, 0, 1);
    }

    //% group="UART"
    //% weight=38
    //% blockGap=8
    //% block="Setup uart B %baud_B bits/sec"
    export function uart_ext_beginB(baud_B: UART_Baud): void {
        if (!initialized) { uart_ext_Initialize(); }
        uart_ext_FIFOEnable(SC16IS752_CHANNEL_B, 1);
        uart_ext_SetBaudrate(SC16IS752_CHANNEL_B, baud_B);
        uart_ext_SetLine(SC16IS752_CHANNEL_B, 8, 0, 1);
    }

    //% group="UART"
    //% weight=37
    //% blockGap=8
    //% block="is uart %channel data available"
    export function uart_ext_available(channel: UART_Ch): boolean {
        if (uart_ext_FIFOAvailableData(channel) != 0) { return true; }

        return false;
    }

    //% group="UART"
    //% weight=36
    //% blockGap=8
    //% block="read uart byte from %channel"
    export function uart_ext_read(channel: UART_Ch): number {
        if (peek_flag[channel] == 0) { return uart_ext_ReadByte(channel); }

        peek_flag[channel] = 0;
        return peek_buf[channel];
    }

    //% group="UART"
    //% weight=35
    //% blockGap=8
    //% block="write uart byte %channel : %val"
    export function uart_ext_write(channel: UART_Ch, val: number): void {
        uart_ext_WriteByte(channel, val);
    }

    //% group="UART"
    //% weight=34
    //% blockGap=8
    //% block="write string %channel : %val"
    export function uart_ext_write_str(channel: UART_Ch, val: string): void {
        for (let i = 0; i < val.length; i++) {
            let code = val.charCodeAt(i);
            uart_ext_WriteByte(channel, code);
        }
    }

    //% group="UART"
    //% weight=33
    //% blockGap=8
    //% block="write buffer %channel : %val"
    export function uart_ext_write_buffer(channel: UART_Ch, val: Buffer): void {
        for (let i = 0; i < val.length; i++) {
            let code = val[i];
            uart_ext_WriteByte(channel, code);
        }
    }

    //% group="UART"
    //% weight=32
    //% blockGap=40
    //% block="write line %channel : %val"
    export function uart_ext_write_line(channel: UART_Ch, val: string): void {
        for (let i = 0; i < val.length; i++) {
            let code = val.charCodeAt(i);
            uart_ext_WriteByte(channel, code);
        }
        uart_ext_WriteByte(channel, 10);
        uart_ext_WriteByte(channel, 13);
    }

    function uart_ext_ReadRegister(channel: number, reg_addr: number): number {
        let result = 0;

        pins.i2cWriteNumber(SC16IS752_addr, (reg_addr << 3 | channel << 1), NumberFormat.UInt8LE, true);
        result = pins.i2cReadNumber(SC16IS752_addr, NumberFormat.UInt8LE, false);

        return result;
    }

    function uart_ext_WriteRegister(channel: number, reg_addr: number, val: number): void {
        let buffer = Buffer.fromArray([(reg_addr << 3 | channel << 1), val]);

        pins.i2cWriteBuffer(SC16IS752_addr, buffer, false);
    }

    function uart_ext_Initialize(): void {
        uart_ext_ResetDevice();
        initialized = true;
    }

    function uart_ext_SetBaudrate(channel: number, baudrate: number): number {
        let divisor;
        let prescaler;
        let actual_baudrate;
        let error;
        let temp_lcr;

        if ((uart_ext_ReadRegister(channel, SC16IS750_REG_MCR) & 0x80) == 0) { prescaler = 1; }
        else { prescaler = 4; }

        divisor = (SC16IS750_CRYSTCAL_FREQ / prescaler) / (baudrate * 16);

        temp_lcr = uart_ext_ReadRegister(channel, SC16IS750_REG_LCR);
        temp_lcr |= 0x80;
        uart_ext_WriteRegister(channel, SC16IS750_REG_LCR, temp_lcr);

        uart_ext_WriteRegister(channel, SC16IS750_REG_DLL, divisor);

        uart_ext_WriteRegister(channel, SC16IS750_REG_DLH, (divisor >> 8));
        temp_lcr &= 0x7F;
        uart_ext_WriteRegister(channel, SC16IS750_REG_LCR, temp_lcr);


        actual_baudrate = (SC16IS750_CRYSTCAL_FREQ / prescaler) / (16 * divisor);
        error = (actual_baudrate - baudrate) * 1000 / baudrate;

        return error;
    }

    function uart_ext_SetLine(channel: number, data_length: number, parity_select: number, stop_length: number): void {
        let temp_lcr;

        temp_lcr = uart_ext_ReadRegister(channel, SC16IS750_REG_LCR);
        temp_lcr &= 0xC0;

        switch (data_length) {
            case 5:
                break;
            case 6:
                temp_lcr |= 0x01;
                break;
            case 7:
                temp_lcr |= 0x02;
                break;
            case 8:
                temp_lcr |= 0x03;
                break;
            default:
                temp_lcr |= 0x03;
                break;
        }

        if (stop_length == 2) {
            temp_lcr |= 0x04;
        }

        switch (parity_select) {
            case 0:
                break;
            case 1:
                temp_lcr |= 0x08;
                break;
            case 2:
                temp_lcr |= 0x18;
                break;
            case 3:
                temp_lcr |= 0x03;
                break;
            case 4:
                break;
            default:
                break;
        }

        uart_ext_WriteRegister(channel, SC16IS750_REG_LCR, temp_lcr);
    }

    function uart_ext_ResetDevice(): void {
        let reg;

        reg = uart_ext_ReadRegister(SC16IS752_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL);
        reg |= 0x08;
        uart_ext_WriteRegister(SC16IS752_CHANNEL_BOTH, SC16IS750_REG_IOCONTROL, reg);
    }

    function uart_ext_FIFOEnable(channel: number, fifo_enable: number): void {
        let temp_fcr;

        temp_fcr = uart_ext_ReadRegister(channel, SC16IS750_REG_FCR);

        if (fifo_enable == 0) { temp_fcr &= 0xFE; }
        else { temp_fcr |= 0x01; }

        uart_ext_WriteRegister(channel, SC16IS750_REG_FCR, temp_fcr);
    }

    function uart_ext_FIFOAvailableData(channel: number): number {
        if (fifo_available[channel] == 0) {
            fifo_available[channel] = uart_ext_ReadRegister(channel, SC16IS750_REG_RXLVL);
        }

        return fifo_available[channel];
    }

    function uart_ext_WriteByte(channel: number, val: number): void {
        let tmp_lsr;

        do {
            tmp_lsr = uart_ext_ReadRegister(channel, SC16IS750_REG_LSR);
        } while ((tmp_lsr & 0x20) == 0);

        uart_ext_WriteRegister(channel, SC16IS750_REG_THR, val);
    }

    function uart_ext_ReadByte(channel: number): number {
        let val;

        if (uart_ext_FIFOAvailableData(channel) == 0) {
            return -1;
        }
        else {
            if (fifo_available[channel] > 0) {
                --fifo_available[channel];
            }

            val = uart_ext_ReadRegister(channel, SC16IS750_REG_RHR);

            return val;
        }
    }

    function uart_ext_readBytes(channel: number, buffer: number[], length: number): number {
        let count = 0;
        let tmp;

        while (count < length) {
            tmp = uart_ext_ReadByte(channel);
            if (tmp < 0) {
                break;
            }
            buffer[count] = tmp;
            count++;
        }
        return count;
    }

    //----------------------------------------------------------------------
    // PCA9685
    //----------------------------------------------------------------------
    /**
     * PCA9685
     */
    // PWM frequency
    const PCA9685_ADDR = 0x7F;
    const PWM_FREQ_HZ = 50;
    let is_PCA9685_init = false;

    function PCA9685_init(freq: number): void {

        if (is_PCA9685_init == true) {
            return;
        }

        is_PCA9685_init = true;

        PCA9685_WriteRegister(0x00, 0x30);                      // Mode register 1, MODE1
        PCA9685_Write2WordRegister(0xFA, 0x0000, 0x1000);       // Disable outputs.

        let scale = (25000000 / (freq * 4096)) - 1;             // Prescale.
        PCA9685_WriteRegister(0xFE, scale);

        // Restart
        PCA9685_WriteRegister(0x00, 0x20);
        control.waitMicros(10000);
        PCA9685_WriteRegister(0x00, 0xA0);

        return;
    }

    function PCA9685_setPWM(channel: number, pwm: number): void {
        if (channel >= 0 && channel <= 15) {
            PCA9685_init(PWM_FREQ_HZ);

            let regCh = 0x06 + (channel * 4);

            if (pwm < 0) { pwm = 0; }
            else if (pwm >= 4095) { pwm = 4095; }

            PCA9685_Write2WordRegister(regCh, 0x0000, pwm);
        }
    }

    function PCA9685_setPulseWidth(channel: number, pulse_us: number): void {
        let pwm = pulse_us * 4096 / 20000;      // 20000us = 1/50Hz
        PCA9685_setPWM(channel, pwm);
    }

    function PCA9685_setDutyCycle(channel: number, percent: number): void {
        let pwm = percent * 4096 / 100;
        PCA9685_setPWM(channel, pwm);
    }

    function PCA9685_setServo(channel: number, degree: number): void {
        let pulse_us = ((degree * 1000) / 180) + 1000;
        PCA9685_setPulseWidth(channel, pulse_us);
    }

    function PCA9685_WriteRegister(reg_addr: number, val: number): void {
        let buffer = pins.createBuffer(2);
        buffer[0] = reg_addr;
        buffer[1] = val;
        pins.i2cWriteBuffer(PCA9685_ADDR, buffer);
    }

    function PCA9685_Write2WordRegister(reg_addr: number, val1: number, val2: number): void {
        let buffer = pins.createBuffer(5);
        buffer[0] = reg_addr;
        buffer[1] = (val1) & 0xff;
        buffer[2] = (val1 >> 8) & 0xff;
        buffer[3] = (val2) & 0xff;
        buffer[4] = (val2 >> 8) & 0xff;
        pins.i2cWriteBuffer(PCA9685_ADDR, buffer);
    }

    //----------------------------------------------------------------------
    // MPU6050
    //----------------------------------------------------------------------
    const MPU6050_ADDR = 0x68;

    const MPU6050_ACC_RATIO = 16384;
    const MPU6050_GYRO_RATIO = 131;

    // Register address
    const MPU6050_ACCX_ADDR = 0x3b;
    const MPU6050_ACCY_ADDR = 0x3d;
    const MPU6050_ACCZ_ADDR = 0x3f;
    const MPU6050_GYROX_ADDR = 0x43;
    const MPU6050_GYROY_ADDR = 0x45;
    const MPU6050_GYROZ_ADDR = 0x47;
    const MPU6050_TEMP_ADDR = 0x41;

    function MPU6050_ReadRegister(reg: number): number {
        pins.i2cWriteNumber(MPU6050_ADDR, reg, NumberFormat.UInt8BE, false);
        return pins.i2cReadNumber(MPU6050_ADDR, NumberFormat.UInt8BE, false);
    }

    function MPU6050_ReadWordRegister(reg: number) {
        let h = MPU6050_ReadRegister(reg);
        let l = MPU6050_ReadRegister(reg + 1);
        let value = (h << 8) + l;

        if (value >= 0x8000) {
            return -((65535 - value) + 1);
        }
        else {
            return value;
        }
    }

    function vlen(a: number, b: number): number {
        return Math.sqrt((a * a) + (b * b));
    }

    /**
     * Initialize SEN-MPU6050
     */

    //% group="IMU Motion sensor"
    //% weight=50
    //% blockGap=8
    //% block="Initial motion sensor"
    export function Motion_init() {
        // Power management
        let buffer = pins.createBuffer(2);
        buffer[0] = 0x6b;
        buffer[1] = 0;
        pins.i2cWriteBuffer(MPU6050_ADDR, buffer);

        // Gyro range
        buffer[0] = 0x1B;
        buffer[1] = 0x00;       // MPU6050_GYRO_FS_250
        pins.i2cWriteBuffer(MPU6050_ADDR, buffer);

        // Acc range
        buffer[0] = 0x1C;
        buffer[1] = 0x00;       // MPU6050_ACCEL_FS_2
        pins.i2cWriteBuffer(MPU6050_ADDR, buffer);
    }

    /**
         * Get gyroscope values
        */
    //%weight=95

    //% group="IMU Motion sensor"
    //% weight=49
    //% blockGap=8
    //% block="Gyroscope value of %axis axis"
    export function Motion_read_rotation(axis: Axis_XYZ): number {
        if (axis == Axis_XYZ.x) {
            return MPU6050_ReadWordRegister(MPU6050_GYROX_ADDR) / MPU6050_GYRO_RATIO;
        }
        else if (axis == Axis_XYZ.y) {
            return MPU6050_ReadWordRegister(MPU6050_GYROY_ADDR) / MPU6050_GYRO_RATIO;
        }
        else {
            return MPU6050_ReadWordRegister(MPU6050_GYROZ_ADDR) / MPU6050_GYRO_RATIO;
        }
    }

    /**
     * Get rotation of the corresponding Axis
     */
    //% group="IMU Motion sensor"
    //% weight=48
    //% blockGap=8
    //% block="Angle of %axis axis"
    export function Motion_read_angle(axis: Axis_XYZ): number {

        let xAcc = MPU6050_ReadWordRegister(MPU6050_ACCX_ADDR) / MPU6050_ACC_RATIO;
        let yAcc = MPU6050_ReadWordRegister(MPU6050_ACCY_ADDR) / MPU6050_ACC_RATIO;
        let zAcc = MPU6050_ReadWordRegister(MPU6050_ACCZ_ADDR) / MPU6050_ACC_RATIO;

        let rad;
        if (axis == Axis_XYZ.x) {
            rad = Math.atan2(yAcc, vlen(xAcc, zAcc));
        }
        else if (axis == Axis_XYZ.y) {
            rad = -Math.atan2(xAcc, vlen(yAcc, zAcc));
        }
        else if (axis == Axis_XYZ.z) {
            rad = Math.atan2(zAcc, vlen(xAcc, yAcc));
        }

        let deg = rad * (180 / Math.PI);
        return deg;
    }

    /**
     * Get acceleration of the corresponding Axis
     */
    //% group="IMU Motion sensor"
    //% weight=47
    //% blockGap=8
    //% block="Acceleration of %axis axis"
    export function Motion_read_acceleration(axis: Axis_XYZ): number {
        if (axis == Axis_XYZ.x) {
            return MPU6050_ReadWordRegister(MPU6050_ACCX_ADDR) / MPU6050_ACC_RATIO;
        }
        else if (axis == Axis_XYZ.y) {
            return MPU6050_ReadWordRegister(MPU6050_ACCY_ADDR) / MPU6050_ACC_RATIO;
        }
        else {
            return MPU6050_ReadWordRegister(MPU6050_ACCZ_ADDR) / MPU6050_ACC_RATIO;
        }
    }

    /**
     * Get temperature
     */
    //% group="IMU Motion sensor"
    //% weight=46
    //% blockGap=8
    //% block="Temperature (Unit: Celsius)"
    export function Read_temperature(): number {
        let rawTemp = MPU6050_ReadWordRegister(MPU6050_TEMP_ADDR);
        return 36.53 + (rawTemp / 340);
    }
}

//--------------------------------------------------------------------------