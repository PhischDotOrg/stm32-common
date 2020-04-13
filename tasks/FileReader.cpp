/*-
 * $Copyright$
-*/

#include "tasks/FileReader.hpp"

#include <string.h>
#include <strings.h>

#if defined(__cplusplus)
extern "C" {
#endif /* defined(__cplusplus) */

#include "FreeRTOS/FreeRTOS.h"
#include "FreeRTOS/task.h"
#include <printf.h>
    
#if defined(__cplusplus)
} /* extern "C" */
#endif /* defined(__cplusplus) */

static FRESULT scan_files(uart::UartDevice *p_uart, char *path)
{
    FRESULT res;
    FILINFO fno;
    DIR dir;
    int i;

    res = pf_opendir(&dir, path);
    if (res != FR_OK)
    {
        p_uart->printf("pf_opendir(%s) failed with rc=0x%.08i\r\n", path, res);
        goto out;
    }

    i = strlen(path);
    while (((res = pf_readdir(&dir, &fno)) == FR_OK) && (fno.fname[0] != 0)) {
        if (fno.fattrib & AM_DIR) {
            p_uart->printf("<D>: %s\r\n", fno.fname);

            tfp_sprintf(&path[i], "/%s", fno.fname);

            res = scan_files(p_uart, path);
            if (res != FR_OK)
                break;

            path[i] = 0;
        } else {
            p_uart->printf("%s/%s\r\n", path, fno.fname);
        }
    }

out:
    return res;
}

namespace tasks {

const char * const FileReader::m_name = "filereader";

/*******************************************************************************
 *
 ******************************************************************************/
FileReader::FileReader(uart::UartDevice *p_uart, fatfs::FatFs *p_fatfs)
  : Task(m_name, 1, 1024), m_uart(p_uart), m_fatfs(p_fatfs) {

}

/*******************************************************************************
 *
 ******************************************************************************/
FileReader::~FileReader() {

}

/*******************************************************************************
 *
 ******************************************************************************/
void
FileReader::run(void) {
    m_uart->printf("Task '%s' starting...\r\n", this->m_name);

    int rc;
    char path[1024];
    bzero(path, sizeof(path));

    rc = this->m_fatfs->mount(path, true);
    if (rc) {
        this->m_uart->printf("Failed to mount FatFS volume! rc=0x%x\r\n", rc);
        goto out;
    }

    scan_files(this->m_uart, path);

    this->m_uart->printf("%s: Done scanning files!\r\n", this->m_name);
out:
    this->m_fatfs->unmount();
}

} /* namespace tasks */