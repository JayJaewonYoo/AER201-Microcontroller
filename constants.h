/* 
 * File:   constants.h
 * Author: Administrator
 *
 * Created on August 11, 2016, 2:41 PM
 */

#ifndef CONSTANTS_H
#define	CONSTANTS_H // Prevents recursive inclusions

/* LCD Control */
#define RS          LATDbits.LATD2 // Register select
#define E           LATDbits.LATD3 // Enable
#define LCD_DELAY   25 // Delay used to comply to datasheet specifications


#endif	/* CONSTANTS_H */

