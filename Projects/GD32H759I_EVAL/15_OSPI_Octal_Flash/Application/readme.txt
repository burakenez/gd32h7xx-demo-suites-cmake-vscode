/*!
    \file    readme.txt
    \brief   description of OSPI 1&8 lines indirect/memory mapped read and write

    \version 2024-07-31, V2.0.0, demo for GD32H7xx
*/

/*
    Copyright (c) 2024, GigaDevice Semiconductor Inc.

    Redistribution and use in source and binary forms, with or without modification, 
are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, this 
       list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright notice, 
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.
    3. Neither the name of the copyright holder nor the names of its contributors 
       may be used to endorse or promote products derived from this software without 
       specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. 
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, 
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT 
NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
OF SUCH DAMAGE.
*/

  This demo is based on the GD32H759I-EVAL V1.0 board, it provides a description of OSPI 
read and write flash with 1&8 lines in indirect/memory mapped mode,  The access result 
will be printed by COM. 
  After system start-up, print some related information and read the id of the flash.
If the id is not equal to GD25X512ME_ID, print the fail information. If not, write and 
read data from the flash. Then check whether the rx_buffer1/rx_buffer2/rx_buffer3 and 
tx_buffer1/tx_buffer2/tx_buffer3 are the same and print the result after that. 

  On the GD32H759I-EVAL-V2.0 board, LED1 connected to PF10, LED2 connected to PA6.
  
  JP68 must be fit USART, JP50, JP66 must be fit to LED, JP61 must be fit to OSPI.
