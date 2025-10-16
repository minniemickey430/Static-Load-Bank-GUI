
Static Load Bank GUI — S2350
Author: Erva Kansu
Date: 2025
------------------------------------------------------------
Purpose:
    This GUI provides a unified interface to monitor and control
    the Static Load Bank drawers integrated in the automated
    test system. It supports:
        - Relay control (static/electronic)
        - Fan mode management (AUTO/MANUAL)
        - NTC sensor acquisition and threshold tracking
        - OLED display control
        - CRC8-ATM framed UART communication (AA..55)
        - Independent watchdog reset recovery
        - Automated CSV/HEX data logging
------------------------------------------------------------
Hardware Context:
    - STM32-based Load Drawer Controller
    - Communication via RS232 (PySerial)
    - Follows custom protocol frame: [AA][ADDR][CMD][LEN][DATA..][CRC][55]
------------------------------------------------------------


