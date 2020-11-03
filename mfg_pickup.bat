set BASE=%CD%

newt build %1
newt create-image stm32l4_boot 0.0.0.1 
newt create-image %1 0.0.0.1
newt mfg create pickup 0.0.0.1
arm-none-eabi-objcopy -I ihex -O ihex --change-addresses=0x08000000 %BASE%\bin\mfgs\pickup\mfgimg.hex %BASE%\hexTargets\pickup.hex