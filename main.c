/**
 *******************************************************************************
 * @file  spi/spi_tx_rx_dma/source/main.c
 * @brief �豸����������������SPI������.
 @verbatim
   Change Logs:
   Date             Author          Notes
   2022-07-29       werech         First version
 @endverbatim
 **/
/**
*******************************************************************************
 * Include files
 ******************************************************************************/
#include "hc32_ddl.h"

/**
 * @addtogroup HC32F4A0_DDL_Examples
 * @{
 */

/**
 * @addtogroup SPI_TX_RX_DMA
 * @{
 */

/*******************************************************************************
 * �ֲ����Ͷ��� ('typedef')
 ******************************************************************************/

/*******************************************************************************
 * ����Ԥ����������/�궨�� ('#define')
 ******************************************************************************/
/* �������� */

/* ����Ӧ�ó����ͨ��ģʽ COM_MASTER or COM_SLAVE*/
#define APP_TEST_MODE           (COM_MASTER)

/* SPI��ģʽ    SPI_WIRE_4/SPI_WIRE_3 */
#define EXAMPLE_WIRE_MODE       (SPI_WIRE_4)

/* SPI ģʽ,  SPI_MODE_0/SPI_MODE_1/SPI_MODE_2/SPI_MODE_3 */
#define EXAMPLE_SPI_MODE        (SPI_MODE_0)

/* ������� SCK_DELAY_FUNC_ON ��Ϊ������ʱ���ӳٺ���*/
#define SCK_DELAY_FUNC_ON       (1UL)

#if (EXAMPLE_WIRE_MODE == SPI_WIRE_3)
    #if ((EXAMPLE_SPI_MODE == SPI_MODE_0) || (EXAMPLE_SPI_MODE == SPI_MODE_2))
        #error "SPI_WIRE_3 mode can't support SPI_MODE_0 and SPI_MODE_2"
    #endif
#endif

#define COM_MASTER              (1UL)
#define COM_SLAVE               (0UL)

/* ����SPI��Ԫ */
#define SPI_UNIT                (M4_SPI3)
#define SPI_PWC_FCG_DEF         (PWC_FCG1_SPI3)
#define SPI_EVT_TX_IRQ          (EVT_SPI3_SPTI)
#define SPI_EVT_RX_IRQ          (EVT_SPI3_SPRI)

/* �˿ڶ��� */
#define SPI_NSS_PORT            (GPIO_PORT_D)
#define SPI_NSS_PIN             (GPIO_PIN_01)
#define SPI_SCK_PORT            (GPIO_PORT_D)
#define SPI_SCK_PIN             (GPIO_PIN_00)
#define SPI_MOSI_PORT           (GPIO_PORT_D)
#define SPI_MOSI_PIN            (GPIO_PIN_03)
#define SPI_MISO_PORT           (GPIO_PORT_D)
#define SPI_MISO_PIN            (GPIO_PIN_02)

#define SPI_NSS_GPIO_FUNC       (GPIO_FUNC_49_SPI3_NSS0)
#define SPI_SCK_GPIO_FUNC       (GPIO_FUNC_46_SPI3_SCK)
#define SPI_MOSI_GPIO_FUNC      (GPIO_FUNC_47_SPI3_MOSI)
#define SPI_MISO_GPIO_FUNC      (GPIO_FUNC_48_SPI3_MISO)

/* DMA ��Ԫ���� */
#define DMA_UNIT                (M4_DMA2)
/* ���ݴ����DMAͨ�� */
#define DMA_CH_TX               (DMA_CH0)
/* ���ݽ��յ�DMAͨ�� */
#define DMA_CH_RX               (DMA_CH1)
/* �ж�Դ��ʾͨ����� */
#define DMA_UNIT_RX_SRC         (INT_DMA2_TC1)
#define DMA_IRQn_RX             (Int000_IRQn)
#define DMA_RX_TC_FLAG          (DMA_TC_INT_CH1)

/* ������Ի��������� */
#define BUF_LENGTH              (128UL)
/*******************************************************************************
 * ȫ�ֱ������� (declared in header file with 'extern')
 ******************************************************************************/

/*******************************************************************************
 * ���غ���ԭ�� ('static')
 ******************************************************************************/
#if (APP_TEST_MODE == COM_MASTER)
static void Master_Init(void);
#else
static void Slave_Init(void);
#endif
/*******************************************************************************
 * �ֲ��������� ('static')
 ******************************************************************************/
static char u8MasterTxBuf[BUF_LENGTH] = "SPI master tx buffer data: abcdefg!";
static char u8SlaveTxBuf[BUF_LENGTH] = "SPI slave tx buffer data: 1234567!";

#if (APP_TEST_MODE == COM_MASTER)
static char u8MasterRxBuf[BUF_LENGTH];
#else
static char u8SlaveRxBuf[BUF_LENGTH];
#endif

static en_flag_status_t enRevFinished = Reset;
/*******************************************************************************
 * ����ʵ�� - ȫ�� ('extern') and �ֲ�('static')
 ******************************************************************************/
/**
 * @brief  MCU ��Χ�Ĵ���д���ޱ���.
 * @param  None
 * @retval None
 * @note Comment/uncomment each API depending on APP requires.
 */
static void Peripheral_WE(void)
{
    /* ����GPIO�Ĵ���: PSPCR, PCCR, PINAER, PCRxy, PFSRxy */
    GPIO_Unlock();
    /* ����PWC �Ĵ���: FCG0 */
    PWC_FCG0_Unlock();
    /* ���� PWC, CLK, PVD �Ĵ���, �� PWC_REG_Write_Unlock_Code*/
    PWC_Unlock(PWC_UNLOCK_CODE_0 | PWC_UNLOCK_CODE_1 | PWC_UNLOCK_CODE_2);
    /* ���� SRAM �Ĵ���: WTCR */
    SRAM_WTCR_Unlock();
    /* ���� SRAM �Ĵ���: CKCR */
    //SRAM_CKCR_Unlock();
    /* �������� EFM �Ĵ��� */
    EFM_Unlock();
    /* ���� EFM �Ĵ���: FWMC */
    //EFM_FWMC_Unlock();
    /* ���� EFM OTP д�����Ĵ��� */
    //EFM_OTP_WP_Unlock();
    /* �������� MPU �Ĵ��� */
    // MPU_Unlock();
}

/**
 * @brief  MCU ��Χ�Ĵ���д����.
 * @param  None
 * @retval None
 * @note Comment/uncomment each API depending on APP requires.
 */
static __attribute__((unused)) void Peripheral_WP(void)
{
    /* Lock GPIO register: PSPCR, PCCR, PINAER, PCRxy, PFSRxy */
    GPIO_Lock();
    /* Lock PWC register: FCG0 */
    PWC_FCG0_Lock();
    /* Lock PWC, CLK, PVD registers, @ref PWC_REG_Write_Unlock_Code for details */
    PWC_Lock(PWC_UNLOCK_CODE_0 | PWC_UNLOCK_CODE_1 | PWC_UNLOCK_CODE_2);
    /* Lock SRAM register: WTCR */
    SRAM_WTCR_Lock();
    /* Lock SRAM register: CKCR */
    //SRAM_CKCR_Lock();
    /* Lock EFM OTP write protect registers */
    //EFM_OTP_WP_Lock();
    /* Lock EFM register: FWMC */
    //EFM_FWMC_Lock();
    /* Lock all EFM registers */
    EFM_Lock();
    /* Lock all MPU registers */
    // MPU_Lock();
}

/**
 * @brief spi_master_base project ������
 * @param  None
 * @retval int32_t return value, if needed
 */
int32_t main(void)
{
    stc_gpio_init_t stcGpioCfg;
#if (APP_TEST_MODE == COM_MASTER)
#if (SCK_DELAY_FUNC_ON == 1UL)
    stc_spi_delay_t stcSpiDelay;
#endif
#endif
    Peripheral_WE();

    BSP_CLK_Init();
    BSP_IO_Init();
    BSP_KEY_Init();
    BSP_LED_Init();

    /* ������ջ����� */
    for(uint32_t i = 0UL; i < BUF_LENGTH; i++)
    {
#if (APP_TEST_MODE == COM_MASTER)
        u8MasterRxBuf[i] = (char)0x00U;
#else
        u8SlaveRxBuf[i] = (char)0x00U;
#endif
    }

    /* ������Χʱ�� */
    PWC_Fcg1PeriphClockCmd(SPI_PWC_FCG_DEF, Enable);
    PWC_Fcg0PeriphClockCmd((PWC_FCG0_DMA2 | PWC_FCG0_AOS), Enable);

    /*�˿����ã�������Ÿߵ�ƽ��High driving capacity�� */
    (void)GPIO_StructInit(&stcGpioCfg);

#if (APP_TEST_MODE == COM_MASTER)
    stcGpioCfg.u16PinDrv = PIN_DRV_HIGH;
#if (EXAMPLE_WIRE_MODE == SPI_WIRE_4)
    (void)GPIO_Init(SPI_NSS_PORT,  SPI_NSS_PIN, &stcGpioCfg);
#endif
    (void)GPIO_Init(SPI_SCK_PORT,  SPI_SCK_PIN, &stcGpioCfg);
    (void)GPIO_Init(SPI_MOSI_PORT, SPI_MOSI_PIN, &stcGpioCfg);

    /*�������ŵ�CMOS���� */
    stcGpioCfg.u16PinIType = PIN_ITYPE_CMOS;
    (void)GPIO_Init(SPI_MISO_PORT, SPI_MISO_PIN, &stcGpioCfg);
#else

    stcGpioCfg.u16PinIType = PIN_ITYPE_CMOS;
#if (EXAMPLE_WIRE_MODE == SPI_WIRE_4)
    (void)GPIO_Init(SPI_NSS_PORT,  SPI_NSS_PIN, &stcGpioCfg);
#endif
    (void)GPIO_Init(SPI_SCK_PORT,  SPI_SCK_PIN, &stcGpioCfg);
    (void)GPIO_Init(SPI_MOSI_PORT, SPI_MOSI_PIN, &stcGpioCfg);

    stcGpioCfg.u16PinDrv = PIN_DRV_HIGH;
    (void)GPIO_Init(SPI_MISO_PORT, SPI_MISO_PIN, &stcGpioCfg);

#endif
    /* �����豸����SPI�˿ں��� */
#if (EXAMPLE_WIRE_MODE == SPI_WIRE_4)
    GPIO_SetFunc(SPI_NSS_PORT,  SPI_NSS_PIN,  SPI_NSS_GPIO_FUNC, PIN_SUBFUNC_DISABLE);
#endif
    GPIO_SetFunc(SPI_SCK_PORT,  SPI_SCK_PIN,  SPI_SCK_GPIO_FUNC, PIN_SUBFUNC_DISABLE);
    GPIO_SetFunc(SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_GPIO_FUNC, PIN_SUBFUNC_DISABLE);
    GPIO_SetFunc(SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_GPIO_FUNC, PIN_SUBFUNC_DISABLE);

    SPI_DeInit(SPI_UNIT);
    /* �����豸����SPI���� */
#if (APP_TEST_MODE == COM_MASTER)
    Master_Init();
#if (SCK_DELAY_FUNC_ON == 1UL)
    /* SPI �ź��ӳ����ã��ȴ��ӻ� */
    (void)SPI_DelayStructInit(&stcSpiDelay);
    stcSpiDelay.u32IntervalDelay = SPI_INTERVAL_TIME_8SCK_2PCLK1;
    stcSpiDelay.u32ReleaseDelay = SPI_RELEASE_TIME_8SCK;
    stcSpiDelay.u32SetupDelay = SPI_SETUP_TIME_8SCK;
    (void)SPI_DelayTimeCfg(SPI_UNIT, &stcSpiDelay);
#endif
#else
    /* �ӻ��豸����SPI����*/
    Slave_Init();
#endif

#if (APP_TEST_MODE == COM_MASTER)
    /* �ȴ��ؼ� */
    while(Pin_Set == GPIO_ReadInputPins(GPIO_PORT_A, GPIO_PIN_00));
#endif
    /* SPI ָ��� */
    SPI_FunctionCmd(SPI_UNIT, Enable);

    /* �ȴ�������� */
    while(Reset == enRevFinished)
    {
        ;
    }

    SPI_DeInit(SPI_UNIT);

#if (APP_TEST_MODE == COM_MASTER)
    /* �Ƚ� u8TxBuffer �� u8RxBuffer */
    if(0 == memcmp(u8SlaveTxBuf, u8MasterRxBuf, BUF_LENGTH))
#else
    if(0 == memcmp(u8MasterTxBuf, u8SlaveRxBuf, BUF_LENGTH))
#endif
    {
        /* ͨ�ųɹ� */
        for(;;)
        {
            BSP_LED_Toggle(LED_BLUE);
            DDL_DelayMS(500UL);
        }
    }
    else
    {
        /* ͨ�����ݴ��� */
        for(;;)
        {
            BSP_LED_Toggle(LED_RED);
            DDL_DelayMS(500UL);
        }
    }
}

/**
 * @brief  DMA �����ж�
 * @param  None
 * @retval None
 */
static void DmaRxEnd_IrqCallback(void)
{
    enRevFinished = Set;
}

#if (APP_TEST_MODE == COM_MASTER)
/**
 * @brief  �������ʼ��
 * @param  None
 * @retval None
 */
static void Master_Init(void)
{
    stc_spi_init_t stcSpiInit;
    stc_dma_init_t stcDmaInit;
    stc_irq_signin_config_t stcIrqSignConfig;

    /* ����SPI���� */
    (void)SPI_StructInit(&stcSpiInit);   /* Clear initialize structure */
    stcSpiInit.u32WireMode          = EXAMPLE_WIRE_MODE;
    stcSpiInit.u32TransMode         = SPI_FULL_DUPLEX;
    stcSpiInit.u32MasterSlave       = SPI_MASTER;
    stcSpiInit.u32SuspMode          = SPI_COM_SUSP_FUNC_OFF;
    stcSpiInit.u32Modfe             = SPI_MODFE_DISABLE;
    stcSpiInit.u32Parity            = SPI_PARITY_INVALID;
    stcSpiInit.u32SpiMode           = EXAMPLE_SPI_MODE;
    stcSpiInit.u32BaudRatePrescaler = SPI_BR_PCLK1_DIV16;
    stcSpiInit.u32DataBits          = SPI_DATA_SIZE_8BIT;
    stcSpiInit.u32FirstBit          = SPI_FIRST_MSB;
    (void)SPI_Init(SPI_UNIT, &stcSpiInit);

    /* TX�����͵�DMA����*/
    DMA_SetTriggerSrc(DMA_UNIT, DMA_CH_TX, SPI_EVT_TX_IRQ);
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn     = DMA_INT_ENABLE;
    stcDmaInit.u32BlockSize = 1UL;
    stcDmaInit.u32TransCnt  = BUF_LENGTH;
    stcDmaInit.u32DataWidth = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32DestAddr  = (uint32_t)(&SPI_UNIT->DR);
    stcDmaInit.u32SrcAddr   = (uint32_t)(&u8MasterTxBuf[0]);
    stcDmaInit.u32SrcInc    = DMA_SRC_ADDR_INC;
    stcDmaInit.u32DestInc   = DMA_DEST_ADDR_FIX;

    if(Ok != DMA_Init(DMA_UNIT, DMA_CH_TX, &stcDmaInit))
    {
        for(;;)
        {
            ;
        }
    }

    /* RX�����յ�DMA����*/
    DMA_SetTriggerSrc(DMA_UNIT, DMA_CH_RX, SPI_EVT_RX_IRQ);
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn     = DMA_INT_ENABLE;
    stcDmaInit.u32BlockSize = 1UL;
    stcDmaInit.u32TransCnt  = BUF_LENGTH;
    stcDmaInit.u32DataWidth = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32DestAddr  = (uint32_t)(&u8MasterRxBuf[0]);
    stcDmaInit.u32SrcAddr   = (uint32_t)(&SPI_UNIT->DR);
    stcDmaInit.u32SrcInc    = DMA_SRC_ADDR_FIX;
    stcDmaInit.u32DestInc   = DMA_DEST_ADDR_INC;

    if(Ok != DMA_Init(DMA_UNIT, DMA_CH_RX, &stcDmaInit))
    {
        for(;;)
        {
            ;
        }
    }

    /* DMA �ж����� */
    stcIrqSignConfig.enIntSrc   = DMA_UNIT_RX_SRC;
    stcIrqSignConfig.enIRQn     = DMA_IRQn_RX;
    stcIrqSignConfig.pfnCallback= &DmaRxEnd_IrqCallback;

    (void)INTC_IrqSignIn(&stcIrqSignConfig);
    DMA_ClearTransIntStatus(DMA_UNIT, DMA_RX_TC_FLAG);

    /* NVIC ���� */
    NVIC_ClearPendingIRQ(DMA_IRQn_RX);
    NVIC_SetPriority(DMA_IRQn_RX,DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(DMA_IRQn_RX);

    /* DMA ģ��ʹ�� */
    DMA_Cmd(DMA_UNIT, Enable);

    /* DMA ͨ��ʹ�� */
    DMA_ChannelCmd(DMA_UNIT, DMA_CH_TX, Enable);
    DMA_ChannelCmd(DMA_UNIT, DMA_CH_RX, Enable);
}

#else

/**
 * @brief  �ӻ���ʼ��
 * @param  None
 * @retval None
 */
static void Slave_Init(void)
{
    stc_spi_init_t stcSpiInit;
    stc_dma_init_t stcDmaInit;
    stc_irq_signin_config_t stcIrqSignConfig;

    (void)SPI_StructInit(&stcSpiInit);   /* �����ʼ���ṹ */
    stcSpiInit.u32WireMode          = EXAMPLE_WIRE_MODE;
    stcSpiInit.u32TransMode         = SPI_FULL_DUPLEX;
    stcSpiInit.u32MasterSlave       = SPI_SLAVE;
    stcSpiInit.u32SuspMode          = SPI_COM_SUSP_FUNC_OFF;
    stcSpiInit.u32Modfe             = SPI_MODFE_DISABLE;
    stcSpiInit.u32Parity            = SPI_PARITY_INVALID;
    stcSpiInit.u32SpiMode           = EXAMPLE_SPI_MODE;
    stcSpiInit.u32BaudRatePrescaler = SPI_BR_PCLK1_DIV2;
    stcSpiInit.u32DataBits          = SPI_DATA_SIZE_8BIT;
    stcSpiInit.u32FirstBit          = SPI_FIRST_MSB;
    (void)SPI_Init(SPI_UNIT, &stcSpiInit);

    /* TX�ӷ��͵�DMA����*/
    DMA_SetTriggerSrc(DMA_UNIT, DMA_CH_TX, SPI_EVT_TX_IRQ);
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn     = DMA_INT_ENABLE;
    stcDmaInit.u32BlockSize = 1UL;
    stcDmaInit.u32TransCnt  = BUF_LENGTH;
    stcDmaInit.u32DataWidth = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32DestAddr  = (uint32_t)(&SPI_UNIT->DR);
    stcDmaInit.u32SrcAddr   = (uint32_t)(&u8SlaveTxBuf[0]);
    stcDmaInit.u32SrcInc    = DMA_SRC_ADDR_INC;
    stcDmaInit.u32DestInc   = DMA_DEST_ADDR_FIX;

    if(Ok != DMA_Init(DMA_UNIT, DMA_CH_TX, &stcDmaInit))
    {
        for(;;)
        {
            ;
        }
    }

    /* RX�ӽ��յ�DMA����*/
    DMA_SetTriggerSrc(DMA_UNIT, DMA_CH_RX, SPI_EVT_RX_IRQ);
    (void)DMA_StructInit(&stcDmaInit);
    stcDmaInit.u32IntEn     = DMA_INT_ENABLE;
    stcDmaInit.u32BlockSize = 1UL;
    stcDmaInit.u32TransCnt  = BUF_LENGTH;
    stcDmaInit.u32DataWidth = DMA_DATAWIDTH_8BIT;
    stcDmaInit.u32DestAddr  = (uint32_t)(&u8SlaveRxBuf[0]);
    stcDmaInit.u32SrcAddr   = (uint32_t)(&SPI_UNIT->DR);
    stcDmaInit.u32SrcInc    = DMA_SRC_ADDR_FIX;
    stcDmaInit.u32DestInc   = DMA_DEST_ADDR_INC;

    if(Ok != DMA_Init(DMA_UNIT, DMA_CH_RX, &stcDmaInit))
    {
        for(;;)
        {
            ;
        }
    }

    /* DMA �ж����� */
    stcIrqSignConfig.enIntSrc   = DMA_UNIT_RX_SRC;
    stcIrqSignConfig.enIRQn     = DMA_IRQn_RX;
    stcIrqSignConfig.pfnCallback= &DmaRxEnd_IrqCallback;

    (void)INTC_IrqSignIn(&stcIrqSignConfig);
    DMA_ClearTransIntStatus(DMA_UNIT, DMA_RX_TC_FLAG);

    /* NVIC����*/
    NVIC_ClearPendingIRQ(DMA_IRQn_RX);
    NVIC_SetPriority(DMA_IRQn_RX,DDL_IRQ_PRIORITY_DEFAULT);
    NVIC_EnableIRQ(DMA_IRQn_RX);

    /* DMA ģ��ʹ�� */
    DMA_Cmd(DMA_UNIT, Enable);

    /* DMA ͨ��ʹ�� */
    DMA_ChannelCmd(DMA_UNIT, DMA_CH_TX, Enable);
    DMA_ChannelCmd(DMA_UNIT, DMA_CH_RX, Enable);
}
#endif

/**
 * @}
 */

/**
 * @}
 */

/*******************************************************************************
 * EOF (not truncated)
 ******************************************************************************/
