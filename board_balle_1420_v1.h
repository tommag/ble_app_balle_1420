/* Header file for board "balle_1420_v1"
 * 
 * Author : Tom Magnier - Cie 14:20
 * 06/2014
 */
 

//Pin definitions
/*#define ACC_INT1_PIN		17						//Accelerometer interrupt line 1
#define ACC_INT2_PIN		18						//Accelerometer interrupt line 2

#define SCL_PIN				14						//I2C SCL pin
#define SDA_PIN				15						//I2C SDA pin

#define CHRG_STAT_PIN		13						//Battery charger IC status pin -- LOW means charging, 

#define WLED_PIN			16						//White LEDS PWM control pin

#define IRLED_PIN			11						//IR LED control pin

#define RX_PIN_NUMBER  		9    					// UART RX pin number
#define TX_PIN_NUMBER  		8    					// UART TX pin number
#define CTS_PIN_NUMBER 		0     					// UART Clear To Send pin number. Not used if HWFC is set to false
#define RTS_PIN_NUMBER 		0     					// Not used if HWFC is set to false
#define HWFC           		false 					// UART hardware flow control


#define BATT_VOLTAGE_AIN_NO	0						//Analog input for battery voltage sensing
*/
#define ACC_INT1_PIN                17						//Accelerometer interrupt line 1
#define ACC_INT2_PIN								18						//Accelerometer interrupt line 2

#define SCL_PIN											14						//I2C SCL pin
#define SDA_PIN											15						//I2C SDA pin

#define CHRG_STAT_PIN								28						//Battery charger IC status pin

#define WLED_PIN										20						//White LEDS PWM control pin

#define IRLED_PIN										22						//IR LED control pin

#define RX_PIN_NUMBER  26    // UART RX pin number
#define TX_PIN_NUMBER  24    // UART TX pin number
#define CTS_PIN_NUMBER 0     // UART Clear To Send pin number. Not used if HWFC is set to false
#define RTS_PIN_NUMBER 0     // Not used if HWFC is set to false
#define HWFC           false // UART hardware flow control


#define BATT_VOLTAGE_AIN_NO					0							//Analog input for battery voltage sensing

/* Header gauche :
3.3V
24 -> TX (white)
26 -> RX (green)
28 -> Chrg status (LOW = charging)
30
16
18 -> Acc Int2
20 -> W LED
22 -> IR LED
GND

Header droite :
x
1
3
5
7
9
11
13
15
GND
*/
