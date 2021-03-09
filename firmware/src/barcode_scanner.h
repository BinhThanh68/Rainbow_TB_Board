
#ifndef _BARCODE_SCANNER_H    /* Guard against multiple inclusion */
#define _BARCODE_SCANNER_H



#ifdef __cplusplus
extern "C" {
#endif

#include "function_initialization.h"
#include <proc/p32mz2048efh064.h>
void Init_Scanner(); 
void Read_Scanner_Data();

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
