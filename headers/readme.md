if these header files are contained within the same directory as the .ino file, the include statement would be as follows:

`#include "disco_... .h"`

if the header flies are contained one folder outside the directory of the .ino file, the include statement would be as follows:

`#include "../disco... .h"`

etc.


<muy importante!!!!>

include the headers in the following order:
`  disco_peripherals.h -> disco_modules.h -> disco_display_modules.h  `


disco_display_modules.h uses functions and globals from disco_modules.h and disco_peripherals.h, respectively.
disco_modules.h uses globals from disco_peripherals.h
