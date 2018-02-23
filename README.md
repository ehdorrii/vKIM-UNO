# vKIM-UNO
6502 emulator for KIM-UNO that includes 6530 emulation

Introducing vKIM-UNO

A few months back I read about Oscar Vermeulen's KIM-UNO, a hardware/software 
emulation of the KIM-1 6502 evaluation board that was a hit with computer
hobbyists (including me) back in the 1970s. I have fond memories of playing
with my KIM-1, and about a dozen years ago I wrote an emulator for it that ran
on the PalmOS -- I called it "vKIM" (for Virtual KIM). You can still find it
on the 'net.

Anyway, Oscar's description of the KIM-UNO was so enticing I couldn't resist.
I ordered one up and eagerly awaited it. It came as a kit, so I dutifully
soldered everything together and promptly tried a program from "The First Book
of KIM". Since you're reading this, I'll assume you already know this was a
wonderful source of KIM-1 programs. Oscar specifically cited this book, and
even included a scan of it as part of the software distribution. I took this
as kind of implying that KIM-UNO would run many if not most of the programs
therein. I tried several, and found that for the most part, only programs that
used the serial interface, or those that used the most basic ROM routines to
interface with the keypad and LED, worked. Even MicroChess only worked with
the serial interface -- and that was not originally supported. To say I was
disappointed is an understatement: I was crushed. What good is a KIM emulation
that doesn't handle the kind of programming that hackers did "back in the day"?

So I decided to dust off my own KIM emulator and shoehorn it into Oscar's
hardware.

While this was mostly fun (I mean, seriously, I probably had more fun crafting
this emulation than you will have using it), there were bits that drove me to 
distraction trying to get working right. It seems that one of the bits of the
KIM-1 hardware that many people seem to ignore is the 6530 timer emulation. In
fact it's really not that difficult. For me, getting the mapping between the 
KIM-1 I/O ports and the Arduino I/O ports correct was a special level of hell 
-- not which pins map to which pins, but how does "active" on the KIM-1 map to
high/low on Arduino pins. Heck, I'm a goddamned programmer, not a hardware
genius. But, I licked that one: I did map all the I/O lines from the 6530s to
the AVR processor and I am not emulating anything I/O-related: I'm mapping it
directly, and the keypad and LEDs are being driven directly by the KIM ROM
firmware.

After that I tried to tackle decimal mode for the ADC and SBC instructions. 
I had punted in my original vKIM emulator, basically citing MOS's inadequate
documentation as a smoke screen to hide behind. Since there's plenty of
documentation on the web now to clear away the ambiguities, I was sure I could
get it "right" this time. After weeks of trying to write code that would pass
Bruce Clark's decimal mode test suite, I've more or less thrown in the towel.
I *think* I'm setting the PS flags correctly, and I'm pretty sure I'm getting
reasonable results for all given inputs ... but I still can't pass Clark's
test suite. 

BUT -- and this is the biggie -- I now have an emulator that runs on Oscar's 
hardware, and runs many of the programs in TFBOK. The ONLY thing that prevents
it from running programs without modification is that it runs *about* 40 times
slower than a real 6502. So anything that depends on timing will -- not 
actually fail, but will run so slow that it *seems* as if it isn't working
properly. Thus, I've made tweaks to any routines that are timing-sensitive. In
particular, I've found that Jim Butterfield (one of the main contributors to 
TFBOK) had a particular idiom that he used over and over -- counting down an 
uninitialized variable in a delay loop (effectively executing said delay loop
256 times), plus *not* de-selecting a digit select line, which causes
"ghosting" of adjacent digits. This is so prevalent in TFBOK that I ended up 
providing a specific "Emulator Trap" (on which, more later) to deal with this
idiom.

So much for background. On to what I am provisionally calling "vKIM-UNO". 
This is just an Arduino sketch that you can load into your KIM-UNO to make it
more faithfully emulate an actual KIM-1. Unlike the 6502 emulator Oscar based
his KIM-1 emulation on, vKIM-UNO will *only* run on Oscar's KIM-UNO. They are
"joined at the hip", so to speak. Once you have loaded it into your KIM-UNO's
Arduino Pro Mini, here's how it works:


0. Turn it on. You'll see the oh-so familiar 0000 00. OK, go ahead and squeal
with delight. You can do everything that you could with the original KIM-1.
Almost. Nothing that utilizes the I/O ports on the second 6530 (so no cassette
tape storage). But pretty much everything else! Ah yes, there is a fly in the
ointment, as it were: it runs *much* slower than the original. This is, after
all, an emulation running on a very low-power (battery-driven!) processor.
Yes, the processor actually runs faster than the 6502 in the original KIM-1,
but it's working really hard to emulate each and every instruction, plus doing
all the overhead stuff like routing all memory accesses to the right place
(RAM, I/O, Timer, ROM), running the 6530 clocks, monitoring the state of the
control lines ... it adds up! 

So anything that depends on timing will either not work at all, or will be
really, really slow. Which explains why I ended up throwing out MicroChess. I
think it would probably work, but it takes *forever* to make a move. OK, it
was awesome back in the day, but you've got plenty of options for a computer
chess opponent these days, right? Besides, isn't Wumpus just the greatest??
Yep, Wumpus works on vKIM-UNO. Nice. Did I mention it's slow? But no worries:
most of the included programs that I've included have been tweaked slightly
to avoid those pesky delay loops. 

The good news is that while the original KIM-1 bit-banged serial I/O using
timing loops, the KIM-UNO is running on an Arduino, which has a *real* serial
port. So serial I/O works great -- much faster than the KIM-1, in fact.


1. When you power on (or reset) the KIM-UNO, holding down a particular button
will have a specific effect as follows:

   "0" -- Sets the KIM-UNO to use the keypad and LED display
   "1" -- Sets the KIM-UNO to use the serial interface
   "2" -- Sets the KIM-UNO to display the 6 KIM digits with a gap: 0000 00
   "3" -- Sets the KIM-UNO to display the 6 KIM digits without a gap: 000000

The last two are useful when running programs that use the display in
different ways. When using the KIM-UNO to examine memory, it's nice to have
that gap, which provides a visual divide between the address and the data at
that address. On the other hand, many TFBOK programs (e.g. Wumpus) assume
there is little or no gap between the digits, and just look better without
the gap.

The effects selected as described above are "remembered". So if (for example)
you select the serial interface, then every time you restart KIM-UNO
afterwards, it will remain in the serial interface "mode", until you switch to
"keypad" mode by restarting while keeping the "0" key depressed. (BTW, since
the display is not driven when in serial mode, if you forget that you started
vKIM-UNO in serial mode the last time you used it, it looks as if it isn't
working anymore. (Oh no! Did I break it? Is the battery dead?) Yes, this
actually happened to me. So when you switch to serial mode the display shows a
distinctly un-KIM-like display of decimal points (. . . . . .).

The remaining keys cause various programs to be pre-loaded into "RAM". When a
program is "pre-loaded" the entry point of the program is also pre-loaded, so
in most cases you need only press GO to begin execution of the program.

   "4" -- Pre-loads TFBOK Asteroids
   "5" -- Pre-loads TFBOK Bagels (no workee)
   "6" -- Pre-loads TFBOK Blackjack 
   "7" -- Pre-loads TFBOK Craps 
   "8" -- Pre-loads TFBOK Lunar lander (no workee)
   "9" -- Pre-loads TFBOK Multi-Maze
   "A" -- Pre-loads TFBOK Wumpus 


2. vKIM-UNO implements an additional instruction that I designate "Emulator
Trap" (EMT). The purpose of EMT is to provide a way for a KIM program to
request a service from the emulator. The opcode for EMT is 0x0F. The second
byte of EMT chooses an operation as follows:

   0F 01 - Write value in A to the serial port
   0F 02 - Query whether a byte is available on the serial port -- Z=0 means
           yes, a value is available; Z=1 means no, there is nothing available.
   0F 03 - Fetch a value from the serial port into A (this is a "blocking"
           call, which means that the calling code will not continue until a
           character is available.)
		
The above are not intended for user programs, since the KIM ROMs provide this
functionality. They are, in fact, how I've patched the ROMs to make those
routines work. A couple of additional EMT operations let you "add" or "remove"
the "TTY jumper" on the fly:

   0F 09 - Use keypad & LED display (equivalent to removing the TTY jumper)
   0F 0A - Use the serial interface (equivalent to installing the TTY jumper)

These let you switch between serial and keypad mode "on the fly", without
having to turn off KIM-UNO (and lose RAM).

	
3. "SST" -- On the KIM-1 the SST switch was a slide-switch that you would
slide to the desired position. On KIM-UNO, SST is a pushbutton. So, SST mode
is toggled each time SST is pressed. To indicate the current setting of SST,
vKIM-UNO uses the decimal point of the rightmost digit: when lit, SST mode is
active; when not lit, SST mode is not active.


4. Saving programs -- The KIM-1 allowed you to save programs in two different
ways: first by punching a range of memory into paper tape (you had an ASR-33
to go with that KIM-1, right?); second, by recording a range of memory onto
magnetic cassette tape. The second option was highly dependent on timing, and
therefore will not work with KIM-UNO (not to mention that the I/O lines this
technique used are unavailable on KIM-UNO). However, you can still save 
programs by "punching paper tape" -- the data is sent to the serial interface,
and most terminal emulators let you capture data to a file. The only tricky
bit is if you did your program development using the keypad and LED display --
in order to switch to serial mode you would normally switch KIM-UNO off, and
then turn it on while holding down the '1' key. Except, of course, now your
program is G-O-N-E. You can switch to serial mode without turning KIM-UNO off
and on though, by using an EMT (emulator trap) instruction. This takes just 3
bytes: 0F 0A 00. (The last byte is a break instruction to return to the KIM
monitor program.) Key these 3 bytes into anyplace in memory, and then execute
and - voila! - you are now in serial mode and can punch your program (see the
KIM User Manual for the details). If you want to switch back to keypad mode
afterwards, use the serial interface to enter and execute the complementary
EMT instruction: 0F 09 00.


5. Loading programs -- Essentially the same technique as described for saving
programs can be used to load programs. Just tell your terminal emulation
program to send the "paper tape" file after entering the serial command L. 



Alas, there remain issues...

* Lunar lander goes to "DEAD 20" immediately. I haven't figured out why.

* Bagels doesn't work. I haven't even started looking at this one yet.

* As far as I can see, the RIOT timers both run continuously (as they are
supposed to), but when using the keypad to examine the value of either timer,
the display shows an unchanging value.

* Decimal mode doesn't pass Bruce Clark's test suite.


But have a pleasant stroll down memory lane! :-)
