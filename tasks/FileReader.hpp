/*-
 * $Copyright$
-*/

#ifndef _FILE_READER_HPP_e3d4c913_74db_4f99_a059_5b66dcff6822
#define _FILE_READER_HPP_e3d4c913_74db_4f99_a059_5b66dcff6822

#include "tasks/Task.hpp"

#include <uart/UartDevice.hpp>
#include <fatfs/FatFs.hpp>
#include <sdcard/sdcard.hpp>

namespace tasks {

class FileReader : public Task {
private:
    static const char * const   m_name;
    uart::UartDevice * const    m_uart;
    fatfs::FatFs * const        m_fatfs;

    virtual void run(void);

public:
    FileReader(uart::UartDevice *p_uart, fatfs::FatFs *p_fatfs);
    virtual ~FileReader();
};

}; /* namespace tasks */

#endif /* _FILE_READER_HPP_e3d4c913_74db_4f99_a059_5b66dcff6822 */
