/*****************************************************************************
* Product: DPP example
* Last Updated for Version: 5.6.4
* Date of the Last Update:  2016-05-09
*
*                    Q u a n t u m     L e a P s
*                    ---------------------------
*                    innovating embedded systems
*
* Copyright (C) Quantum Leaps, LLC. All rights reserved.
*
* This program is open source software: you can redistribute it and/or
* modify it under the terms of the GNU General Public License as published
* by the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* Alternatively, this program may be distributed and modified under the
* terms of Quantum Leaps commercial licenses, which expressly supersede
* the GNU General Public License and are specifically designed for
* licensees interested in retaining the proprietary status of their code.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program. If not, see <http://www.gnu.org/licenses/>.
*
* Contact information:
* http://www.state-machine.com
* mailto:info@state-machine.com
*****************************************************************************/
#if !defined( bsp_h ) /**< Include Guard                          */
#define bsp_h
    
#ifdef __cplusplus   //macro do deal with C++ mangling prevention
# define __BEGIN_DECLS extern "C" {
# define __END_DECLS }
#else
# define __BEGIN_C_DECLS /* empty */
# define __END_C_DECLS /* empty */
#endif    

#define BSP_TICKS_PER_SEC    1000U
#define SYSTICK_INTERRUPT_VECTOR_NUMBER 15u /* Cortex-M3 hard vector DO NOT CHANGE FROM 15 */
    
__BEGIN_C_DECLS

#define BSP_LedOff(); 		pinLED_Write(0);
#define BSP_LedOn(); 		pinLED_Write(1);	
#define BSP_LedToggle(); 	pinLED_Write(~pinLED_Read());
//#define BSP_Shutdown(); 	LED15p4_Write(0);
    
  // tried this alternate way:
//inline void BSP_LedToggle(void){ pinLED_Write(~pinLED_Read()); }
    
void BSP_init(void);
void BSP_displayPaused(uint8_t paused);
void BSP_displayPhilStat(uint8_t n, char_t const *stat);
void BSP_terminate(int16_t result);
void assert_failed(char const *module, int loc);


void BSP_randomSeed(uint32_t seed);   /* random seed */
uint32_t BSP_random(void);            /* pseudo-random generator */

/* for testing... */

__END_C_DECLS

#endif /* bsp_h */
