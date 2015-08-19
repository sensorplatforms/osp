/* mbed Microcontroller Library
 * Copyright (c) 2006-2013 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
//#include "mbed_assert.h"
#include "i2c_api.h"

#include "cmsis.h"
#include "board.h"
#include "hw_setup_nxp_lpc54102.h"
#include "clock_5410x.h"
#include "syscon_5410x.h"
#include "pinmap.h"

#define I2C_MEM_SZ      (64) /* Size of memory for I2C Slave ROM driver */

/* I2CM transfer record */
static  ROM_I2CM_XFER_T  i2cmXferRec[DEVICE_I2CSLAVE];
i2c_t   *pi2c_slave_obj[DEVICE_I2CSLAVE] = {NULL};

/*****************************************************************************
 * Private functions
 ****************************************************************************/

#ifdef I2CM_IRQ_BASED
/* Function to wait for I2CM transfer completion */
static void WaitForI2cXferComplete(I2CM_XFER_T *xferRecPtr)
{
    /* Are we still transferring data ? */
    while (xferRecPtr->status == I2CM_STATUS_BUSY) 
    {
        /* Sleep until next interrupt */
        __WFI();
    }
}
#endif

static void i2cslave_start(ROM_I2CS_HANDLE_T hI2C, uint16_t addr)
{
    /* For DBG:Do nothing here, enable below code to find the object corresponding to this handle */
#if 0
    uint8_t  ui8_index = 0;
    i2c_t *obj = NULL;

    for(ui8_index=0;ui8_index<DEVICE_I2CSLAVE;ui8_index++)
    {
        if((NULL != pi2c_slave_obj[ui8_index]) 
                && (hI2C == pi2c_slave_obj[ui8_index]->px_slave_handle))
        {
            obj = pi2c_slave_obj[ui8_index];
            break;
        }
    }

    obj->i2csStarted = 1;
#endif
}

/* I2C Tx [Will be called when a byte is ready to be received;
 * It also resets bytesSend to 0 */
static ROM_I2CS_TRANCTRL_T i2cslave_tx( ROM_I2CS_HANDLE_T   hI2C, 
                                        ROM_I2CS_XFER_T    *pXfer)
{ 
    static volatile int finished = 0;  
    uint8_t  ui8_index = 0;
    i2c_t *obj = NULL;

    for(ui8_index=0;ui8_index<DEVICE_I2CSLAVE;ui8_index++)
    {
        if((NULL != pi2c_slave_obj[ui8_index]) 
                && (hI2C == pi2c_slave_obj[ui8_index]->px_slave_handle))
        {
            obj = pi2c_slave_obj[ui8_index];
            obj->i2c_operation = 0x2;
            break;
        }
    }

    /* Send until we reach the specified region in the tx buffer */
    pXfer->txBuff = (uint8_t *)pXfer->txBuff + pXfer->txSz;
    //D0_printf("I2CS bytes sent %d from %x\r\n", pXfer->txSz,pXfer->txBuff);
    return ROM_I2CS_CONTINUE;
}
static ROM_I2CS_TRANCTRL_T i2cslave_rx( ROM_I2CS_HANDLE_T hI2C, 
                                        ROM_I2CS_XFER_T *pXfer)
{
    static volatile int finished = 0;  
    uint8_t  ui8_index = 0;
    i2c_t *obj = NULL;

    //D0_printf("I2CS bytes to recv %d to %x\r\n", pXfer->rxSz,pXfer->rxBuff);
    for(ui8_index=0;ui8_index<DEVICE_I2CSLAVE;ui8_index++)
    {
        if((NULL != pi2c_slave_obj[ui8_index]) 
                && (hI2C == pi2c_slave_obj[ui8_index]->px_slave_handle))
        {
            obj = pi2c_slave_obj[ui8_index];
            obj->i2c_operation = 0x1;
            break;
        }
    }
    pXfer->rxBuff = (uint8_t *)pXfer->rxBuff + pXfer->rxSz;
    pXfer->rxSz++;

    //D0_printf("RxCB rxBuff = 0x%p,  rxSz = %d\r\n", pXfer->rxBuff, pXfer->rxSz);
    return ROM_I2CS_CONTINUE;
}


/* I2C Done event callback [Will be called after STOP bit is received] */
static void i2cslave_done(ROM_I2CS_HANDLE_T hI2C, ROM_I2CS_XFER_T *pXfer)
{
    uint8_t  ui8_index = 0;
    i2c_t *obj;

    for(ui8_index=0;ui8_index<DEVICE_I2CSLAVE;ui8_index++)
    {
        if((NULL != pi2c_slave_obj[ui8_index]) 
                && (hI2C == pi2c_slave_obj[ui8_index]->px_slave_handle))
        {
            obj = pi2c_slave_obj[ui8_index];
            break;
        }
    }
    //D0_printf("I2CS Stopped\r\n");
    obj->pXfer.rxBuff = obj->rxBuff;
    obj->pXfer.rxSz = 0;
    ROM_I2CS_Transfer(hI2C, &obj->pXfer);
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
void i2c_init(i2c_t *obj, PinName sda, PinName scl) 
{
    static bool                init = false;
    /* Create more instance of below 2 variables if more then a slave and master are used */
    static uint32_t            devMem[I2C_MEM_SZ];
    static uint32_t            i2cmContextData[16];
    uint32_t                   memSize;
    uint8_t                    ui8_index = 0;
    LPC_I2C_T                  *px_i2c_base = 0;
    CHIP_SYSCON_CLOCK_T        e_clock = SYSCON_CLOCK_I2C0;
    CHIP_SYSCON_PERIPH_RESET_T periph = RESET_I2C0;
#if defined(I2CM_DMA_BASED)
    DMA_CHID_T                 e_dma_ch = DMAREQ_I2C0_MASTER;
#endif /* I2CM_DMA_BASED */

    if(obj->i2c == I2C_LPC_0)
    {
        e_clock = SYSCON_CLOCK_I2C0;
        periph = RESET_I2C0;
        px_i2c_base = LPC_I2C0;
    }
    else if(obj->i2c == I2C_LPC_1)
    {
        e_clock = SYSCON_CLOCK_I2C1;
        periph = RESET_I2C1;
        px_i2c_base = LPC_I2C1;
    }
    else if(obj->i2c == I2C_LPC_2)
    {
        e_clock = SYSCON_CLOCK_I2C2;
        periph = RESET_I2C2;
        px_i2c_base = LPC_I2C2;
    }
    /* Enable I2C clock and reset I2C peripheral */
    Chip_Clock_EnablePeriphClock(e_clock);
    Chip_SYSCON_PeriphReset(periph);

    if((0 != sda) && (0 != scl))
    {
        /* Slave init */
        ROM_I2CS_INIT_T i2csInit;

        /* Setup I2C pin mux */
        pin_function( ENCODE_PORT_PIN((uint8_t)Port_0, (uint8_t)Pin_27), (PINMAP_FUNC1 | PINMAP_DIGITAL_EN));
        pin_function( ENCODE_PORT_PIN((uint8_t)Port_0, (uint8_t)Pin_28), (PINMAP_FUNC1 | PINMAP_DIGITAL_EN));
        memSize = ROM_I2CS_GetMemSize();
        if ( memSize > sizeof(devMem) ) 
        {
            /* should not happen, Increase I2CS_MEM_SZ */
            D0_printf("ERROR: Insufficient I2C device memory buffer.\r\n");
            while(1);
        }

        i2csInit.pUserData = NULL;
        i2csInit.base      = (uint32_t) px_i2c_base;
        obj->px_slave_handle = ROM_I2CS_Init(devMem, &i2csInit);
        for(ui8_index=0;ui8_index<DEVICE_I2CSLAVE;ui8_index++)
        {
            if(NULL == pi2c_slave_obj[ui8_index])
            {
                pi2c_slave_obj[ui8_index] = obj;
                break;
            }
        }
        ROM_I2CS_RegisterCallback(obj->px_slave_handle, ROM_I2CS_START_CB, 
          (void*) i2cslave_start);  // slave start 
        ROM_I2CS_RegisterCallback(obj->px_slave_handle, ROM_I2CS_XFERRECV_CB, 
                                    (void*) i2cslave_rx);
        ROM_I2CS_RegisterCallback(obj->px_slave_handle, ROM_I2CS_XFERSEND_CB, 
                                    (void*) i2cslave_tx); // From master
        ROM_I2CS_RegisterCallback(obj->px_slave_handle, ROM_I2CS_DONE_CB, 
                                    (void*) i2cslave_done); // slave xfer complete

    }
    else
    {
        if (init == false) 
        {
            ROM_I2CM_INIT_T i2cmInit;

            /* Initialize driver */
            i2cmInit.pUserData = NULL;
            i2cmInit.base = (uint32_t)px_i2c_base;
            obj->px_master_handle = ROM_I2CM_Init(i2cmContextData, &i2cmInit);
            if (obj->px_master_handle == NULL) 
            {
                /* Error initializing I2C */
                /* FIXME implement some sort of error indicator */
            } 

            /* Setup I2CM transfer rate */
            ROM_I2CM_SetClockRate(obj->px_master_handle, 
                      Chip_Clock_GetAsyncSyscon_ClockRate(), I2C_MCLOCK_SPEED);

#if defined(I2CM_DMA_BASED)
            /* Setup I2C0 slave channel for the following configuration:
               - Peripheral DMA request
               - Single transfer
               - High channel priority */
            Chip_DMA_EnableChannel(LPC_DMA, I2C_SENSOR_BUS_DMAID);
            Chip_DMA_EnableIntChannel(LPC_DMA, I2C_SENSOR_BUS_DMAID);
            Chip_DMA_SetupChannelConfig(LPC_DMA, I2C_SENSOR_BUS_DMAID,
                                        (DMA_CFG_PERIPHREQEN 
                                                | DMA_CFG_TRIGBURST_SNGL 
                                                | DMA_CFG_CHPRIORITY(0)));
#endif /* I2CM_DMA_BASED */

#ifdef I2CM_IRQ_BASED
            {
                I2C0_IRQn e_irq_type = I2C0_IRQn; //Initialise to default value

                if(obj->i2c == I2C_LPC_0)
                {
                    e_irq_type = I2C0_IRQn;
                }

                NVIC_SetPriority(e_irq_type, obj->ui_irq_priority);
                /* Enable the interrupt for the I2C */
                NVIC_EnableIRQ(e_irq_type);
            }
#endif
            init = true;
        }
    }
    return;

}

int i2c_read(i2c_t *obj, int address, char *data, int length, int stop)
{    
    memset(&i2cmXferRec[obj->i2c], 0, sizeof(i2cmXferRec[obj->i2c]));
    /* Setup I2C transfer record */
    i2cmXferRec[obj->i2c].slaveAddr = address;
    i2cmXferRec[obj->i2c].status = LPC_OK;
    i2cmXferRec[obj->i2c].txSz = 1;
    i2cmXferRec[obj->i2c].rxSz = length;
    i2cmXferRec[obj->i2c].txBuff = obj->tx_buff;
    i2cmXferRec[obj->i2c].rxBuff = data;

#ifdef I2CM_IRQ_BASED
    /* Start transfer and wait for completion */
    ROM_I2CM_Transfer(obj->px_master_handle, &i2cmXferRec[obj->i2c]);
    /* Wait for transfer completion */
    WaitForI2cXferComplete(&i2cmXferRec[obj->i2c]);
#else
    /* I2C master driver will block if blocking flag is used */
    i2cmXferRec[obj->i2c].flags = ROM_I2CM_FLAG_BLOCKING;

    /* Start transfer and wait for completion */
    ROM_I2CM_Transfer(obj->px_master_handle, &i2cmXferRec[obj->i2c]);
#endif

    return 0;
}

int i2c_write(i2c_t *obj, int address, const char *data, int length, int stop)
{
    memset(&i2cmXferRec[obj->i2c], 0, sizeof(i2cmXferRec[obj->i2c]));

    i2cmXferRec[obj->i2c].slaveAddr = address;
    i2cmXferRec[obj->i2c].status = LPC_OK;
    i2cmXferRec[obj->i2c].txSz = length+1;
    i2cmXferRec[obj->i2c].rxSz = 0;
    i2cmXferRec[obj->i2c].txBuff = data;
    i2cmXferRec[obj->i2c].rxBuff = 0;

#ifdef I2CM_IRQ_BASED
    /* Start transfer and wait for completion */
    ROM_I2CM_Transfer(obj->px_master_handle, &i2cmXferRec[obj->i2c]);
    /* Wait for transfer completion */
    WaitForI2cXferComplete(&i2cmXferRec[obj->i2c]);
#else
    /* I2C master driver will block if blocking flag is used */
    i2cmXferRec[obj->i2c].flags = ROM_I2CM_FLAG_BLOCKING;

    /* Start transfer and wait for completion */
    ROM_I2CM_Transfer(obj->px_master_handle, &i2cmXferRec[obj->i2c]);
#endif    
    return 0;

}

#if DEVICE_I2CSLAVE
void i2c_slave_mode(i2c_t *obj, int enable_slave)
{
    ROM_I2CS_SLAVE_T slaveSetup;
    uint32_t         optimalDev;

    /* Setup slave address to respond to */
    slaveSetup.slaveAddr = obj->ui_slave_address;     /* host address is 7 bit */
    slaveSetup.SlaveIndex = obj->uc_slave_index;
    slaveSetup.EnableSlave = enable_slave;
    ROM_I2CS_SetupSlave(obj->px_slave_handle, &slaveSetup);

    /* Setup the clock rate for I2C. */
    optimalDev = Chip_Clock_GetAsyncSyscon_ClockRate()/4000000;  /* 250ns */
    if(obj->i2c == I2C_LPC_2)
    {
        LPC_I2C2->CLKDIV = optimalDev;
    }

    /* D0_printf("I2C Slave address: 0x%x, CLOCK_DIV: %d\r\n", 
                           obj->ui_slave_address, optimalDev); */

    /* Start first transfer, this would also register the Rx buffer 
        * for receiving the bytes from Host  */
    obj->pXfer.rxBuff = obj->rxBuff;
    ROM_I2CS_Transfer(obj->px_slave_handle, &(obj->pXfer));

}

/*This function could be blocking till an operation is requested from the host.
 * Currently this is being called from IRQ handler.
 */
int  i2c_slave_receive(i2c_t *obj)
{
    obj->i2c_operation = 0;
    ROM_I2CS_TransferHandler(obj->px_slave_handle);
    return obj->i2c_operation;
}
int  i2c_slave_read   (i2c_t *obj, char *data, int length)
{
    obj->rxLength = length;
    if (obj->rxCount >= obj->rxLength) 
    {
        obj->rxCount = 0;
        obj->rxLength = 0;
        obj->pXfer.rxSz  = 0;
        /* next i2c incoming data start from beginning of the buffer.  */
        obj->pXfer.rxBuff = obj->rxBuff;
    }
    memcpy((void *)data,(void *)obj->rxBuff,obj->rxLength);
    return (int)obj->rxLength;
}
int  i2c_slave_write  (i2c_t *obj, const char *data, int length)
{
    obj->pXfer.txBuff = (uint8_t *)data;
    obj->pXfer.txSz = length;
    /* Return success now, bytes will be written when master is requesting for data */
    return length;
}
int  i2c_slave_byte_read(i2c_t *obj, int last)
{
    obj->rxLength = 1;
    if (obj->rxCount >= obj->rxLength) 
    {
        obj->rxCount = 0;
        obj->rxLength = 0;
        obj->pXfer.rxSz  = 0;
        /* next i2c incoming data start from beginning of the buffer.  */
        obj->pXfer.rxBuff = obj->rxBuff;
    }
    return	obj->rxBuff[0];
}
int  i2c_slave_byte_write(i2c_t *obj, int data)
{
    obj->pXfer.txBuff = (uint8_t *)data;
    obj->pXfer.txSz = 1;
    /* Return success now, bytes will be written when master is requesting for data */
    return 1;
}
void i2c_slave_address(i2c_t *obj, int idx, uint32_t address, uint32_t mask)
{
    return;
}
#endif

