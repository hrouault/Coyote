/*
 * =====================================================================================
 *
 *       Filename:  i2c.c
 *
 *    Description:
 *
 *        Version:  1.0
 *        Created:  08.12.2011 17:35:13
 *       Revision:  none
 *       Compiler:  gcc
 *
 *         Author:  YOUR NAME (),
 *        Company:
 *
 * =====================================================================================
 */

/*******************************************************************************
* Function Name  : CODEC_WriteRegister
* Description    : Writes a value in a register of the LCD display through I2C.
* Input          :  - RegisterAddr: The target register adress (between 00x and 0x24)
*                :  - RegisterValue: The target register value to be written
*                :  - Verify: 0-> Don't verify the written data, 1-> Verify the written data
* Output         : None
* Return         : - 0  -> Correct write operation
*                : - !0 -> Incorrect write operation
*******************************************************************************/
uint32_t CODEC_WriteRegister(uint32_t RegisterAddr, uint32_t RegisterValue)
{
    uint32_t read_verif = 0;
    /* Reset all I2C2 registers */
    I2C_SoftwareResetCmd(I2C1, ENABLE);
    I2C_SoftwareResetCmd(I2C1, DISABLE);
    /* Enable the I2C1 peripheral  */
    I2C_Cmd(I2C1, ENABLE);
    /* Configure the I2C peripheral */
    I2C_Config();
    /* Begin the config sequence */
    I2C_GenerateSTART(I2C1, ENABLE);
    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT))
    {}
    /* Transmit the slave address and enable writing operation */
    I2C_Send7bitAddress(I2C1, I2CINTER_ADDRESS, I2C_Direction_Transmitter);
    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED))
    {}
    /* Transmit the first address for r/w operations */
    I2C_SendData(I2C1, RegisterAddr);
    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {}
    /* Prepare the register value to be sent */
    I2C_SendData(I2C1, RegisterValue);
    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED))
    {}
    /* End the configuration sequence */
    I2C_GenerateSTOP(I2C1, ENABLE);
    /* Verify (if needed) that the loaded data is correct  */
#ifdef VERIFY_WRITTENDATA
    /* Read the just written register*/
    read_verif = CODEC_ReadRegister(RegisterAddr);
    /* Load the register and verify its value  */
    if (read_verif != RegisterValue) {
        /* Control data wrongly tranfered */
        read_verif = 1;
    } else {
        /* Control data correctly transfered */
        read_verif = 0;
    }
#endif
    /* Return the verifying value: 0 (Passed) or 1 (Failed) */
    return read_verif;
}
