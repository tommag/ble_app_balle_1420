/* Header file for board "balle_1420_v1"
 * 
 * Author : Tom Magnier - Cie 14:20
 * 06/2014
 */
 

//Pin definitions
/*#define ACC_INT1_PIN                17						//Accelerometer interrupt line 1
#define ACC_INT2_PIN								18						//Accelerometer interrupt line 2

#define SCL_PIN											14						//I2C SCL pin
#define SDA_PIN											15						//I2C SDA pin

#define CHRG_STAT_PIN								13						//Battery charger IC status pin

#define WLED_PIN										16						//White LEDS PWM control pin

#define IRLED_PIN										11						//IR LED control pin

#define TX_PIN											8							//Debug TTY TX pin
#define RX_PIN											9							//Debug TTY RX pin

#define BATT_VOLTAGE_AIN_NO					0							//Analog input for battery voltage sensing
*/
#define ACC_INT1_PIN                17						//Accelerometer interrupt line 1
#define ACC_INT2_PIN								18						//Accelerometer interrupt line 2

#define SCL_PIN											14						//I2C SCL pin
#define SDA_PIN											15						//I2C SDA pin

#define CHRG_STAT_PIN								28						//Battery charger IC status pin

#define WLED_PIN										20						//White LEDS PWM control pin

#define IRLED_PIN										22						//IR LED control pin

#define TX_PIN											24							//Debug TTY TX pin
#define RX_PIN											26							//Debug TTY RX pin

#define BATT_VOLTAGE_AIN_NO					0							//Analog input for battery voltage sensing

/* Header gauche :
3.3V
24 -> TX
26 -> RX
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