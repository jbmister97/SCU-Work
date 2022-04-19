This ReadMe.txt file will change with the git checkins. THe info about each stage will show up here in this doc.

First version:

You should jump TX/RX on the board (JP6) and see your characters come back to you from YAT (or your terminal of choice).

After that, you should connect the two pins on JP6 to D0/D1 (RX to TX) and with the added code in stm32g0xx_it.c (we will call 
this the _it.c file for short) you should see the echo come back to you:

Line 28/29:
#include "stm32g0xx_hal.h"
#include <stm32g0xx_ll_usart.h>

Line 155-157:
if(LL_USART_IsActiveFlag_RXNE(USART1) && LL_USART_IsEnabledIT_RXNE(USART1)) {
  LL_USART_TransmitData8(USART1, LL_USART_ReceiveData8(USART1));
}


Note the things added to the main.c file to enable this:

Line 46: 
char receiveBuffer[25] = 0;

Line 94:
HAL_UART_Receive_IT(&huart1, receiveBuffer, 25);

The _IT version of functions enables the interrupt (assumes in CubeMX you have enabled interrupts for the peripheral).

When that's working, move onto the next checkin.


-------------- Second Level Project ----------------------------
This version adds the scheduler, and adds a simple comm buffer for in and out. 
Still, the buffers are used only for echoing back characters, but this is done  by
putting received char in a buffer, and using a function to pull char out of the buffer and send them back. 

This fuction will be modified in future versions to process commands.

Note the following changes in main.c, _it.c, and the inclusion of serial.c (and the use of schedulder.h)

Take a look at scheduler.h. You will see some macros defined. We get used to seeing constants defined:

#define FRED 23

but these are "macro substitutions" which means whenerver the compiler sees "FRED" it 
substitutes "23" (not the number, but the characters BEFORE the tokenizing).

so, that means we can substitute anything for anything. there are "editing" rules for more 
advanced substitutions, like multi-line. You see the use of that in the defines for the code that
goes into the _it.c file. This allows us to make very readable code. It is effective if you need 
code "inline" (not a callable function) and it's not too hard to maintain if the code is stable.

We make use of that here in implementing the scheduler. Using the scheduler.h file contents as copy/paste 
and macros, you can set up a scheduler in under 5 minutes - well worth the time.

Take a look at the code in the UART interrupt handler in the _it.c file, and the includes, #defines,
and privte variables at the top. Also not the additions in includes and variables in the main.c file. 

A serial_user.c/h file pair is also added to the module. this holds the processiing functions for 
when characters come in. In this current case it is very simple: read it out of the input buffer and put it 
in the output buffer.

The buffers are managed in the _it.c file uart handler. as things come in they go into the input buffer.

If there is anything to go out, and the output register is empty, it is placed in there. when the
output buffer is empty, the interrupt is turned off.

To set up the recieve interrupt, look at line 104 in main.c. This is a "dummy" buffer variable that is
never used. when you run it, you can see in the watch window nothing ever goes in there. We control
this by putting our code BEFORE the built-in handler for the interrupt. 

There are two variables declared in serial.c: nextSerialRxIn and nextSerialRx2Proc. in main.c, 
lines 150 and 151 use these two variable to determine if there are unprocessed received 
characters in the buffer, and, if so, calls the processing function. As mentioned before, the simple
processing function we've created simply copies the character out of the RX buffer and inserts it into 
the tx buffer.


You can test it using the YAT terminal file in the project directory (you will likely have to 
change the com # when it opens, but just select the ST comm port in the list).

--------------- AFTER YOU TEST -------------
When you look at the new function calls added, you will notice that some of the functions 
don't start with "HAL_" but start with "LL_." These are Low-Level drivers and are sometimes more 
efficent or give more granular control. You see them being used in the interrupt handlers.

Remember the function call on line 104? If you comment out that line and uncomment line 105, you will 
find that the dummy variable goes away (even if you don't comment out it's declaration it won't be 
in the watch window becsue it is optomised out because it's not being used anywhere). 
The code still works, because we are using one of these LL_ calls to turn on the interrupt.

You will notice that we did NOT have to include the LL_...usart.c file in the build - none of those
functions are being used. They are actually located in the LL_ header file as macros.


-------------- Third Level Project ----------------------------
In this commit, the buffer-handling code is changed to macros making it easier to read.

The processing code has been changed to implement a simple protocol, and the protocol commands
have been put into the YAT termial "preset messages" area.

The protocol looka like this:
$<CMD>[<PARAM>]<LF>

1) The "sync" char is a $
2) Command char is second, and for this protocol can be upper or lower case: R, S, T, U V, W, X
3) if the command requires a parameter, it is in ascii following the <CMD>
4) The packet is terminated with a <LF> ('\n')
(An additional <<CR> is added in YAT so it looks good when youa re reading it.) 

The commands use the on-board LED and Blue User Button

You can see the commands in the code, but for now:
R: Turn on the LED
S: Turn Off the LED
T: Toggle the LED
U: Flash the LED at a speed (this requires a parameter)
V: Report if the button was ever pressed (and reset the status to "not pressed")
W: Write a value into a variable (this requires a parameter
X: Flash the LED at 0.5hz

Note how the ProcessReceiveBuffer() and ProcessPacket() functions have changed.

The processing in the _it.c file is now handled with macros as we did with the scheduler - this 
makes it easier to read. 

--------------- TEST IT OUT -------------
The commands have all been loaded in the predefined buttons in the YAT Termial. The important 
variables have been added to the live watch. You can see the result of the commands on the board and 
in the watch window. If you hit the tiop button "This is a Test" you will see the input buffer being 
changed (it turns red for a sec) but notice that the packet buffer is NOT updated, because there is no 
"$" in the string, so the packet capture is not triggered.

--------------- AFTER YOU TEST -------------
Note that there is a warning generated. Look at lines 87 and 102 in "serial_user.c." They are basically 
the same function call, but one is generating a warnign and one is not, yet they both work fine. This
is because the type expected by the function is not the same type being passed (pointer to uint_8 vs. 
pointer to char). It works because they are the same format and size, but we have eliminated the warning 
by casting the parameter on line 87 with a (char const *) which is what the function is expecting.

Go ahead and fix that on line 102.

