#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/interrupt.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/rom_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "sensorlib/hw_mpu9150.h"
#include "sensorlib/hw_ak8975.h"
#include "sensorlib/i2cm_drv.h"
#include "sensorlib/ak8975.h"
#include "sensorlib/mpu9150.h"
#include "sensorlib/comp_dcm.h"
#include "drivers/buttons.h"
#include "drivers/pinout.h"
#include "stdio.h"
#include <stdlib.h>


//*****************************************************************************
//
// Define MPU9150 I2C Address.
//
//*****************************************************************************
#define MPU9150_I2C_ADDRESS     0x68

//*****************************************************************************
//
// Global variable for holder the actual system clock speed.
//
//*****************************************************************************
uint32_t g_ui32SysClock;

//*****************************************************************************
//
// Global instance structure for the I2C master driver.
//
//*****************************************************************************
tI2CMInstance g_sI2CInst;

//*****************************************************************************
//
// Global instance structure for the ISL29023 sensor driver.
//
//*****************************************************************************
tMPU9150 g_sMPU9150Inst;

//*****************************************************************************
//
// Global Instance structure to manage the DCM state.
//
//*****************************************************************************
tCompDCM g_sCompDCMInst;

//*****************************************************************************
//
// Global flags to alert main that MPU9150 I2C transaction is complete
//
//*****************************************************************************
volatile uint_fast8_t g_vui8I2CDoneFlag;

//*****************************************************************************
//
// Global flags to alert main that MPU9150 I2C transaction error has occurred.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8ErrorFlag;

//*****************************************************************************
//
// Global flags to alert main that MPU9150 data is ready to be retrieved.
//
//*****************************************************************************
volatile uint_fast8_t g_vui8DataFlag;

//*****************************************************************************
//
// Global counter to control and slow down the rate of data to the terminal.
//
//*****************************************************************************
#define PRINT_SKIP_COUNT        10
uint32_t g_ui32PrintSkipCounter;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// MPU9150 Sensor callback function.  Called at the end of MPU9150 sensor
// driver transactions. This is called from I2C interrupt context. Therefore,
// we just set a flag and let main do the bulk of the computations and display.
//
//*****************************************************************************
void
MPU9150AppCallback(void *pvCallbackData, uint_fast8_t ui8Status)
{
    //
    // Turn off the LED to show that transaction is complete.
    //
    LEDWrite(CLP_D3 | CLP_D4, 0);

    //
    // If the transaction succeeded set the data flag to indicate to
    // application that this transaction is complete and data may be ready.
    //
    if(ui8Status == I2CM_STATUS_SUCCESS)
    {
        g_vui8I2CDoneFlag = 1;
    }

    //
    // Store the most recent status in case it was an error condition
    //
    g_vui8ErrorFlag = ui8Status;
}

//*****************************************************************************
//
// Called by the NVIC as a result of GPIO port M interrupt event. For this
// application GPIO port M pin 3 is the interrupt line for the MPU9150
//
// For BoosterPack 2 Interface use Port M pin 7.
//
//*****************************************************************************
void
GPIOPortMIntHandler(void)
{
    unsigned long ulStatus;

    //
    // Get the status flags to see which pin(s) caused the interrupt.
    //
    ulStatus = GPIOIntStatus(GPIO_PORTM_BASE, true);

    //
    // Clear all the pin interrupts that are set
    //
    GPIOIntClear(GPIO_PORTM_BASE, ulStatus);

    //
    // Check if this is an interrupt on the MPU9150 interrupt line.
    //
    // For BoosterPack 2 use Pin 7 instead.
    //
    if(ulStatus & GPIO_PIN_3)
    {
        //
        // Turn on the LED to show that transaction is starting.
        //
        LEDWrite(CLP_D3 | CLP_D4, CLP_D3);

        //
        // MPU9150 Data is ready for retrieval and processing.
        //
        MPU9150DataRead(&g_sMPU9150Inst, MPU9150AppCallback, &g_sMPU9150Inst);
    }
}

//*****************************************************************************
//
// Called by the NVIC as a result of I2C7 Interrupt. I2C7 is the I2C connection
// to the MPU9150.
//
//
// For BoosterPack 2 interface change this to the I2C8 interrupt location in
// the vector table in the startup file.
//
//*****************************************************************************
void
MPU9150I2CIntHandler(void)
{
    //
    // Pass through to the I2CM interrupt handler provided by sensor library.
    // This is required to be at application level so that I2CMIntHandler can
    // receive the instance structure pointer as an argument.
    //
    I2CMIntHandler(&g_sI2CInst);
}

//*****************************************************************************
//
// MPU9150 Application error handler. Show the user if we have encountered an
// I2C error.
//
//*****************************************************************************
void
MPU9150AppErrorHandler(char *pcFilename, uint_fast32_t ui32Line)
{
    uint32_t ui32LEDState;

    //
    // Set terminal color to red and print error status and locations
    //
    UARTprintf("\033[31;1m");
    UARTprintf("Error: %d, File: %s, Line: %d\n"
               "See I2C status definitions in utils\\i2cm_drv.h\n",
               g_vui8ErrorFlag, pcFilename, ui32Line);

    //
    // Return terminal color to normal
    //
    UARTprintf("\033[0m");

    //
    // Read the initial LED state and clear the CLP_D3 LED
    //
    LEDRead(&ui32LEDState);
    ui32LEDState &= ~CLP_D3;

    //
    // Do nothing and wait for interventions.  A more robust application could
    // attempt corrective actions here.
    //
    while(1)
    {
        //
        // Toggle LED 4 to indicate the error.
        //
        ui32LEDState ^= CLP_D4;
        LEDWrite(CLP_D3 | CLP_D4, ui32LEDState);

        //
        // Do Nothing
        //
        ROM_SysCtlDelay(g_ui32SysClock / (10 * 3));
    }
}
//*****************************************************************************
//
// Function to wait for the MPU9150 transactions to complete. Use this to spin
// wait on the I2C bus.
//
//*****************************************************************************
void
MPU9150AppI2CWait(char *pcFilename, uint_fast32_t ui32Line)
{
    //
    // Put the processor to sleep while we wait for the I2C driver to
    // indicate that the transaction is complete.
    //
    while((g_vui8I2CDoneFlag == 0) && (g_vui8ErrorFlag == 0))
    {
        //
        // Do Nothing
        //
    }

    //
    // If an error occurred call the error handler immediately.
    //
    if(g_vui8ErrorFlag)
    {
        MPU9150AppErrorHandler(pcFilename, ui32Line);
    }

    //
    // clear the data flag for next use.
    //
    g_vui8I2CDoneFlag = 0;
}

//*****************************************************************************
//
// Configure the UART and its pins.  This must be called before UARTprintf().
//
//*****************************************************************************
void
ConfigureUART(void)
{
    //
    // Enable the GPIO Peripheral used by the UART.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Enable UART0
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Configure GPIO Pins for UART mode.
    //
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 9600, g_ui32SysClock);
}

//*****************************************************************************
//
// Main application entry point.
//
//*****************************************************************************
int
main(void)
{
		int_fast32_t i32IPart[16], i32FPart[16];
    uint_fast32_t ui32Idx, ui32CompDCMStarted;
    float pfData[16];
    float *pfAccel;// *pfGyro, *pfMag, *pfEulers, *pfQuaternion;
		// Initialize convenience pointers that clean up and clarify the code
    // meaning. We want all the data in a single contiguous array so that
    // we can make our pretty printing easier later.	
		pfAccel = pfData;
		// Configure the system frequency.
		g_ui32SysClock = MAP_SysCtlClockFreqSet((SYSCTL_XTAL_25MHZ |
                                             SYSCTL_OSC_MAIN | SYSCTL_USE_PLL |
                                             SYSCTL_CFG_VCO_480), 120000000);
		//
    // Configure the device pins for this board.
    //
    PinoutSet(false, false);

    //
    // Initialize the UART.
    //
    ConfigureUART();

    //
    // Print the welcome message to the terminal.
    //
    UARTprintf("\033[2JMPU9150 Complimentary DCM Example\n");

        //
    // The I2C7 peripheral must be enabled before use.
    //
    // For BoosterPack 2 interface use I2C8.
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C7);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);

    //
    // Configure the pin muxing for I2C7 functions on port D0 and D1.
    // This step is not necessary if your part does not support pin muxing.
    //
    // For BoosterPack 2 interface use PA2 and PA3.
    //
    ROM_GPIOPinConfigure(GPIO_PD0_I2C7SCL);
    ROM_GPIOPinConfigure(GPIO_PD1_I2C7SDA);

    //
    // Select the I2C function for these pins.  This function will also
    // configure the GPIO pins pins for I2C operation, setting them to
    // open-drain operation with weak pull-ups.  Consult the data sheet
    // to see which functions are allocated per pin.
    //
    // For BoosterPack 2 interface use PA2 and PA3.
    //
    GPIOPinTypeI2CSCL(GPIO_PORTD_BASE, GPIO_PIN_0);
    ROM_GPIOPinTypeI2C(GPIO_PORTD_BASE, GPIO_PIN_1);

    //
    // Configure and Enable the GPIO interrupt. Used for INT signal from the
    // MPU9150.
    //
    // For BoosterPack 2 interface change this to PM7.
    //
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTM_BASE, GPIO_PIN_3);
    GPIOIntEnable(GPIO_PORTM_BASE, GPIO_PIN_3);
    ROM_GPIOIntTypeSet(GPIO_PORTM_BASE, GPIO_PIN_3, GPIO_FALLING_EDGE);
    ROM_IntEnable(INT_GPIOM);

    //
    // Keep only some parts of the systems running while in sleep mode.
    // GPIOM is for the MPU9150 interrupt pin.
    // UART0 is the virtual serial port
    // I2C7 is the I2C interface to the ISL29023
    //
    // For BoosterPack 2 Interface change this to I2C8.
    //
    ROM_SysCtlPeripheralClockGating(true);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOM);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_I2C7);

    //
    // Enable interrupts to the processor.
    //
    ROM_IntMasterEnable();

    //
    // Initialize I2C7 peripheral.
    //
    // For BoosterPack 2 use I2C8.
    //
    I2CMInit(&g_sI2CInst, I2C7_BASE, INT_I2C7, 0xff, 0xff, g_ui32SysClock);

    //
    // Turn on the LED to show a transaction is starting.
    //
    LEDWrite(CLP_D3 | CLP_D4, CLP_D3);

    //
    // Initialize the MPU9150 Driver.
    //
    MPU9150Init(&g_sMPU9150Inst, &g_sI2CInst, MPU9150_I2C_ADDRESS,
                MPU9150AppCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MPU9150AppI2CWait(__FILE__, __LINE__);

    //
    // Turn on the LED to show a transaction is starting.
    //
    LEDWrite(CLP_D3 | CLP_D4, CLP_D3);

    //
    // Write application specifice sensor configuration such as filter settings
    // and sensor range settings.
    //
    g_sMPU9150Inst.pui8Data[0] = MPU9150_CONFIG_DLPF_CFG_94_98;
    g_sMPU9150Inst.pui8Data[1] = MPU9150_GYRO_CONFIG_FS_SEL_250;
    g_sMPU9150Inst.pui8Data[2] = (MPU9150_ACCEL_CONFIG_ACCEL_HPF_5HZ |
                                  MPU9150_ACCEL_CONFIG_AFS_SEL_2G);
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_CONFIG, g_sMPU9150Inst.pui8Data, 3,
                 MPU9150AppCallback, &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MPU9150AppI2CWait(__FILE__, __LINE__);

    //
    // Turn on the LED to show a transaction is starting.
    //
    LEDWrite(CLP_D3 | CLP_D4, CLP_D3);

    //
    // Configure the data ready interrupt pin output of the MPU9150.
    //
    g_sMPU9150Inst.pui8Data[0] = MPU9150_INT_PIN_CFG_INT_LEVEL |
                                    MPU9150_INT_PIN_CFG_INT_RD_CLEAR |
                                    MPU9150_INT_PIN_CFG_LATCH_INT_EN;
    g_sMPU9150Inst.pui8Data[1] = MPU9150_INT_ENABLE_DATA_RDY_EN;
    MPU9150Write(&g_sMPU9150Inst, MPU9150_O_INT_PIN_CFG,
                 g_sMPU9150Inst.pui8Data, 2, MPU9150AppCallback,
                 &g_sMPU9150Inst);

    //
    // Wait for transaction to complete
    //
    MPU9150AppI2CWait(__FILE__, __LINE__);

    //
    // Initialize the DCM system. 50 hz sample rate.
    // accel weight = .2, gyro weight = .8, mag weight = .2
    //
    CompDCMInit(&g_sCompDCMInst, 1.0f / 50.0f, 0.2f, 0.6f, 0.2f);

    //
    // Print the basic outline of our data table. Done once and then kept as we
    // print only the data.
    //
    //UARTprintf("\033[2J\033[H");
    //UARTprintf("MPU9150 9-Axis Simple Data Application Example\n\n");
    //UARTprintf("\033[20GX\033[31G|\033[43GY\033[54G|\033[66GZ\n\n");
    //UARTprintf("Accel\033[8G|\033[31G|\033[54G|\n\n");
    //UARTprintf("Gyro\033[8G|\033[31G|\033[54G|\n\n");
    //UARTprintf("Mag\033[8G|\033[31G|\033[54G|\n\n");
    //UARTprintf("\n\033[20GRoll\033[31G|\033[43GPitch\033[54G|\033[66GYaw\n\n");
    //UARTprintf("Eulers\033[8G|\033[31G|\033[54G|\n\n");

    //UARTprintf("\n\033[17GQ1\033[26G|\033[35GQ2\033[44G|\033[53GQ3\033[62G|"
    //           "\033[71GQ4\n\n");
    //UARTprintf("Q\033[8G|\033[26G|\033[44G|\033[62G|\n\n");

    ui32CompDCMStarted = 0;

    while(1)
    {
        //
        // Go to sleep mode while waiting for data ready.
        //
        while(!g_vui8I2CDoneFlag)
        {
            ROM_SysCtlSleep();
        }

        //
        // Clear the flag
        //
        g_vui8I2CDoneFlag = 0;

        //
        // Get floating point version of the Accel Data in m/s^2.
        //
        MPU9150DataAccelGetFloat(&g_sMPU9150Inst, pfAccel, pfAccel + 1,
                                 pfAccel + 2);

       
        //
        // Check if this is our first data ever.
        //
        if(ui32CompDCMStarted == 0)
        {
            //
            // Set flag indicating that DCM is started.
            // Perform the seeding of the DCM with the first data set.
            //
            ui32CompDCMStarted = 1;
         //   CompDCMMagnetoUpdate(&g_sCompDCMInst, pfMag[0], pfMag[1],
         //                        pfMag[2]);
            CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],
                               pfAccel[2]);
         //   CompDCMGyroUpdate(&g_sCompDCMInst, pfGyro[0], pfGyro[1],
         //                     pfGyro[2]);
            CompDCMStart(&g_sCompDCMInst);
        }
        else
        {
            //
           
            CompDCMAccelUpdate(&g_sCompDCMInst, pfAccel[0], pfAccel[1],
                               pfAccel[2]);
           
            CompDCMUpdate(&g_sCompDCMInst);
        }

        //
        // Increment the skip counter.  Skip counter is used so we do not
        // overflow the UART with data.
        //
        g_ui32PrintSkipCounter++;
        if(g_ui32PrintSkipCounter >= PRINT_SKIP_COUNT)
        {
            //
            // Reset skip counter.
            //
            g_ui32PrintSkipCounter = 0;

           
            for(ui32Idx = 0; ui32Idx < 16; ui32Idx++)
            {
                //
                // Conver float value to a integer truncating the decimal part.
                //
                i32IPart[ui32Idx] = (int32_t) pfData[ui32Idx];

                //
                // Multiply by 1000 to preserve first three decimal values.
                // Truncates at the 3rd decimal place.
                //
                i32FPart[ui32Idx] = (int32_t) (pfData[ui32Idx] * 100.0f);

                //
                // Subtract off the integer part from this newly formed decimal
                // part.
                //
                i32FPart[ui32Idx] = i32FPart[ui32Idx] -
                                    (i32IPart[ui32Idx] * 1000);

                //
                // make the decimal part a positive number for display.
                //
                if(i32FPart[ui32Idx] < 0)
                {
                    i32FPart[ui32Idx] *= -1;
                }
            }
						
            //
            // Print the acceleration numbers in the table.
            //
            UARTprintf("\033[5;17H%3d", i32IPart[0]);
            UARTprintf("\033[5;40H%3d", i32IPart[1]);
            UARTprintf("\033[5;63H%3d", i32IPart[2]);
						UARTprintf("\n");
						UARTprintf("\n");
						//
						//	Writing code for right,left,stop,back forward
						//
						//while(1)
							//{
								if (i32IPart[2]==9)
								{
									UARTprintf("STOP    \n");
								}
								else if (i32IPart[0]==-9|i32IPart[0]==-10)
								{
									UARTprintf("LEFT    \n");
								}
								else if (i32IPart[0]==9)
								{
									UARTprintf("RIGHT    \n");
								}
								else if (i32IPart[1]==5|i32IPart[2]==8)
								{
									UARTprintf("FORWARD \n");
								}
								else if (i32IPart[1]==-5|i32IPart[2]==8)
								{
									UARTprintf("BACKWARD\n");
								}
								//else
									//{
									//UARTprintf("STOP1\n");
								//}
							//}
					

        }
    }

}
