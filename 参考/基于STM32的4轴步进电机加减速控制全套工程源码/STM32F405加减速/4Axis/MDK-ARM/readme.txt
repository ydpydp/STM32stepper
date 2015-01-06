/**
  @page mdkarm MDK-ARM Project Template for STM32F4xx devices
  
  @verbatim
  ******************** (C) COPYRIGHT 2011 STMicroelectronics *******************
  * @file    readme.txt
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    30-September-2011
  * @brief   This sub-directory contains all the user-modifiable files needed to 
  *          create a new project linked with the STM32F4xx Standard Peripherals  
  *          Library and working with RealView Microcontroller Development Kit(MDK-ARM)
  *          software toolchain (Version 4.22 and later).
  ******************************************************************************
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  ******************************************************************************
  @endverbatim
 
 @par Directory contents
 
 - Project.uvproj/.uvopt: A pre-configured project file with the provided library 
                          structure that produces an executable image with MDK-ARM.

@note Enabling "Options for Target — Output – Browser Information" is useful for
      quick source files navigation but may slow the compilation time.                 
 
 
 @par How to use it ?
 
 - Open the Project.uvproj project
 - Rebuild all files: Project->Rebuild all target files
 - Load project image: Debug->Start/Stop Debug Session
 - Run program: Debug->Run (F5)

 @note The needed define symbols for this config are already declared in the
       preprocessor section: USE_STM324xG_EVAL, STM32F4XX, USE_STDPERIPH_DRIVER 
    
 * <h3><center>&copy; COPYRIGHT 2011 STMicroelectronics</center></h3>
 */
