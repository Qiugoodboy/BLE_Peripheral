  open and config uart 
  halUARTCfg_t uartConfig;
  uartConfig.configured        =TRUE;
  uartConfig.baudRate          =HAL_UART_BR_115200;
  uartConfig.flowControl       =FALSE;
  uartConfig.callBackFunc      =UART_CB;
  uartConfig.rx.maxBufSize     =SBP_UART_RX_BUF_SIZE;
  uartConfig.tx.maxBufSize     =SBP_UART_TX_BUF_SIZE;
  uartConfig.intEnable         =TRUE;
  
  HalUARTOpen(0,&uartConfig);
    UartInit();