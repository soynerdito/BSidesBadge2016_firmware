# BSidesBadge2016_firmware
Work in progress for BSides Badge 2016. Using Keil arm development. STM32F0 ARM Cortex M0 

Project made with Keil on windows.
However code can be compiled with SystemWorkbench or other Arm M0 compiler that supports HAL libraries.

Note:
This is not a coding modification, just in my case from linux downloading code via STLinkV2.

In my case, to make SystemWorkbench work in my installation I had to change the .cfg file to adjust the reset option
just change it to this:
# use hardware reset, connect under reset
reset_config srst_only srst_nogate
