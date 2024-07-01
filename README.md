# Cortex-M4-NOP-MOV-Cache-test   
Testing NOP duration, MOV R0 R0 duration, and Cache benefits on STM32   
  
The test consists of CubeMX-generated main.c, with clock and cache configured using CubeMX.  
  
After that, I use SysTick to measure the number of cycles it takes for certain actions.  
I also used disassembly to estimate how long certain instructions take.  
I tried to be careful with it, disabled interrupts and exceptions of the CPU, used memory and instruction barriers.   
  
The test was performed on STM32L433 (Nucleo board), various frequencies.    

Test 1:  
100x NOP instruction in a row (no loop). Testing if the pipeline arbitrarily drops any NOPs out of the pipeline as per ARM documentation (= allowed to drop).  
Result: all 100 NOP instructions were executed (consistently).  

Test 2:  
100x MOV R0 R0 instructions in a ron (no loop), which is an alternative NOP as per ARM. Same test, checking if the pipeline arbitrarily drops it.  
Result: all 100 MOV R0, R0 instructions were executed (consistently).  
  
Test 3:  
Instruction cache test.   
Result: With CPU at 4MHz (0 Flash wait states), instruction cache or prefetch buffer have no performance effect. With CPU at 64MHz (Flash has 3 wait states),  
enabling at least one of the prefetch buffer or instruction cache, maxed out the performace (100 cycles for 100 NOPs). With instruction cache and prefetch buffer both disabled,  
the penalty was 25% (100 NOPs took 125 cycles).  
  
Test 4:  
D-Cache test.  
Result: Turned out D-Cache only works with data from the Flash, it doesn't care about RAM reads and writes (like I thought it would).  
Still, learned that a single byte read takes as much time as a single 32-bit read (3 clock cycles, consistently).   
Enabling or disabling D-cache has no effect on this.  
