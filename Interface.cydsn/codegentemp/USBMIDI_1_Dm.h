/*******************************************************************************
* File Name: USBMIDI_1_Dm.h  
* Version 2.20
*
* Description:
*  This file contains Pin function prototypes and register defines
*
* Note:
*
********************************************************************************
* Copyright 2008-2015, Cypress Semiconductor Corporation.  All rights reserved.
* You may use this file only in accordance with the license, terms, conditions, 
* disclaimers, and limitations in the end user license agreement accompanying 
* the software package with which this file was provided.
*******************************************************************************/

#if !defined(CY_PINS_USBMIDI_1_Dm_H) /* Pins USBMIDI_1_Dm_H */
#define CY_PINS_USBMIDI_1_Dm_H

#include "cytypes.h"
#include "cyfitter.h"
#include "cypins.h"
#include "USBMIDI_1_Dm_aliases.h"

/* APIs are not generated for P15[7:6] */
#if !(CY_PSOC5A &&\
	 USBMIDI_1_Dm__PORT == 15 && ((USBMIDI_1_Dm__MASK & 0xC0) != 0))


/***************************************
*        Function Prototypes             
***************************************/    

/**
* \addtogroup group_general
* @{
*/
void    USBMIDI_1_Dm_Write(uint8 value);
void    USBMIDI_1_Dm_SetDriveMode(uint8 mode);
uint8   USBMIDI_1_Dm_ReadDataReg(void);
uint8   USBMIDI_1_Dm_Read(void);
void    USBMIDI_1_Dm_SetInterruptMode(uint16 position, uint16 mode);
uint8   USBMIDI_1_Dm_ClearInterrupt(void);
/** @} general */

/***************************************
*           API Constants        
***************************************/
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup driveMode Drive mode constants
     * \brief Constants to be passed as "mode" parameter in the USBMIDI_1_Dm_SetDriveMode() function.
     *  @{
     */
        #define USBMIDI_1_Dm_DM_ALG_HIZ         PIN_DM_ALG_HIZ
        #define USBMIDI_1_Dm_DM_DIG_HIZ         PIN_DM_DIG_HIZ
        #define USBMIDI_1_Dm_DM_RES_UP          PIN_DM_RES_UP
        #define USBMIDI_1_Dm_DM_RES_DWN         PIN_DM_RES_DWN
        #define USBMIDI_1_Dm_DM_OD_LO           PIN_DM_OD_LO
        #define USBMIDI_1_Dm_DM_OD_HI           PIN_DM_OD_HI
        #define USBMIDI_1_Dm_DM_STRONG          PIN_DM_STRONG
        #define USBMIDI_1_Dm_DM_RES_UPDWN       PIN_DM_RES_UPDWN
    /** @} driveMode */
/** @} group_constants */
    
/* Digital Port Constants */
#define USBMIDI_1_Dm_MASK               USBMIDI_1_Dm__MASK
#define USBMIDI_1_Dm_SHIFT              USBMIDI_1_Dm__SHIFT
#define USBMIDI_1_Dm_WIDTH              1u

/* Interrupt constants */
#if defined(USBMIDI_1_Dm__INTSTAT)
/**
* \addtogroup group_constants
* @{
*/
    /** \addtogroup intrMode Interrupt constants
     * \brief Constants to be passed as "mode" parameter in USBMIDI_1_Dm_SetInterruptMode() function.
     *  @{
     */
        #define USBMIDI_1_Dm_INTR_NONE      (uint16)(0x0000u)
        #define USBMIDI_1_Dm_INTR_RISING    (uint16)(0x0001u)
        #define USBMIDI_1_Dm_INTR_FALLING   (uint16)(0x0002u)
        #define USBMIDI_1_Dm_INTR_BOTH      (uint16)(0x0003u) 
    /** @} intrMode */
/** @} group_constants */

    #define USBMIDI_1_Dm_INTR_MASK      (0x01u) 
#endif /* (USBMIDI_1_Dm__INTSTAT) */


/***************************************
*             Registers        
***************************************/

/* Main Port Registers */
/* Pin State */
#define USBMIDI_1_Dm_PS                     (* (reg8 *) USBMIDI_1_Dm__PS)
/* Data Register */
#define USBMIDI_1_Dm_DR                     (* (reg8 *) USBMIDI_1_Dm__DR)
/* Port Number */
#define USBMIDI_1_Dm_PRT_NUM                (* (reg8 *) USBMIDI_1_Dm__PRT) 
/* Connect to Analog Globals */                                                  
#define USBMIDI_1_Dm_AG                     (* (reg8 *) USBMIDI_1_Dm__AG)                       
/* Analog MUX bux enable */
#define USBMIDI_1_Dm_AMUX                   (* (reg8 *) USBMIDI_1_Dm__AMUX) 
/* Bidirectional Enable */                                                        
#define USBMIDI_1_Dm_BIE                    (* (reg8 *) USBMIDI_1_Dm__BIE)
/* Bit-mask for Aliased Register Access */
#define USBMIDI_1_Dm_BIT_MASK               (* (reg8 *) USBMIDI_1_Dm__BIT_MASK)
/* Bypass Enable */
#define USBMIDI_1_Dm_BYP                    (* (reg8 *) USBMIDI_1_Dm__BYP)
/* Port wide control signals */                                                   
#define USBMIDI_1_Dm_CTL                    (* (reg8 *) USBMIDI_1_Dm__CTL)
/* Drive Modes */
#define USBMIDI_1_Dm_DM0                    (* (reg8 *) USBMIDI_1_Dm__DM0) 
#define USBMIDI_1_Dm_DM1                    (* (reg8 *) USBMIDI_1_Dm__DM1)
#define USBMIDI_1_Dm_DM2                    (* (reg8 *) USBMIDI_1_Dm__DM2) 
/* Input Buffer Disable Override */
#define USBMIDI_1_Dm_INP_DIS                (* (reg8 *) USBMIDI_1_Dm__INP_DIS)
/* LCD Common or Segment Drive */
#define USBMIDI_1_Dm_LCD_COM_SEG            (* (reg8 *) USBMIDI_1_Dm__LCD_COM_SEG)
/* Enable Segment LCD */
#define USBMIDI_1_Dm_LCD_EN                 (* (reg8 *) USBMIDI_1_Dm__LCD_EN)
/* Slew Rate Control */
#define USBMIDI_1_Dm_SLW                    (* (reg8 *) USBMIDI_1_Dm__SLW)

/* DSI Port Registers */
/* Global DSI Select Register */
#define USBMIDI_1_Dm_PRTDSI__CAPS_SEL       (* (reg8 *) USBMIDI_1_Dm__PRTDSI__CAPS_SEL) 
/* Double Sync Enable */
#define USBMIDI_1_Dm_PRTDSI__DBL_SYNC_IN    (* (reg8 *) USBMIDI_1_Dm__PRTDSI__DBL_SYNC_IN) 
/* Output Enable Select Drive Strength */
#define USBMIDI_1_Dm_PRTDSI__OE_SEL0        (* (reg8 *) USBMIDI_1_Dm__PRTDSI__OE_SEL0) 
#define USBMIDI_1_Dm_PRTDSI__OE_SEL1        (* (reg8 *) USBMIDI_1_Dm__PRTDSI__OE_SEL1) 
/* Port Pin Output Select Registers */
#define USBMIDI_1_Dm_PRTDSI__OUT_SEL0       (* (reg8 *) USBMIDI_1_Dm__PRTDSI__OUT_SEL0) 
#define USBMIDI_1_Dm_PRTDSI__OUT_SEL1       (* (reg8 *) USBMIDI_1_Dm__PRTDSI__OUT_SEL1) 
/* Sync Output Enable Registers */
#define USBMIDI_1_Dm_PRTDSI__SYNC_OUT       (* (reg8 *) USBMIDI_1_Dm__PRTDSI__SYNC_OUT) 

/* SIO registers */
#if defined(USBMIDI_1_Dm__SIO_CFG)
    #define USBMIDI_1_Dm_SIO_HYST_EN        (* (reg8 *) USBMIDI_1_Dm__SIO_HYST_EN)
    #define USBMIDI_1_Dm_SIO_REG_HIFREQ     (* (reg8 *) USBMIDI_1_Dm__SIO_REG_HIFREQ)
    #define USBMIDI_1_Dm_SIO_CFG            (* (reg8 *) USBMIDI_1_Dm__SIO_CFG)
    #define USBMIDI_1_Dm_SIO_DIFF           (* (reg8 *) USBMIDI_1_Dm__SIO_DIFF)
#endif /* (USBMIDI_1_Dm__SIO_CFG) */

/* Interrupt Registers */
#if defined(USBMIDI_1_Dm__INTSTAT)
    #define USBMIDI_1_Dm_INTSTAT            (* (reg8 *) USBMIDI_1_Dm__INTSTAT)
    #define USBMIDI_1_Dm_SNAP               (* (reg8 *) USBMIDI_1_Dm__SNAP)
    
	#define USBMIDI_1_Dm_0_INTTYPE_REG 		(* (reg8 *) USBMIDI_1_Dm__0__INTTYPE)
#endif /* (USBMIDI_1_Dm__INTSTAT) */

#endif /* CY_PSOC5A... */

#endif /*  CY_PINS_USBMIDI_1_Dm_H */


/* [] END OF FILE */
