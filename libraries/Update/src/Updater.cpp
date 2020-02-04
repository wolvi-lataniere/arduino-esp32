#include "Update.h"
#include "Arduino.h"
#include "esp_spi_flash.h"
#include "esp_ota_ops.h"
#include "esp_image_format.h"

#define MIN(A,B) ((A<B)?A:B)

static const char * _err2str(uint8_t _error){
    if(_error == UPDATE_ERROR_OK){
        return ("No Error");
    } else if(_error == UPDATE_ERROR_WRITE){
        return ("Flash Write Failed");
    } else if(_error == UPDATE_ERROR_ERASE){
        return ("Flash Erase Failed");
    } else if(_error == UPDATE_ERROR_READ){
        return ("Flash Read Failed");
    } else if(_error == UPDATE_ERROR_SPACE){
        return ("Not Enough Space");
    } else if(_error == UPDATE_ERROR_SIZE){
        return ("Bad Size Given");
    } else if(_error == UPDATE_ERROR_STREAM){
        return ("Stream Read Timeout");
    } else if(_error == UPDATE_ERROR_MD5){
        return ("MD5 Check Failed");
    } else if(_error == UPDATE_ERROR_MAGIC_BYTE){
        return ("Wrong Magic Byte");
    } else if(_error == UPDATE_ERROR_ACTIVATE){
        return ("Could Not Activate The Firmware");
    } else if(_error == UPDATE_ERROR_NO_PARTITION){
        return ("Partition Could Not be Found");
    } else if(_error == UPDATE_ERROR_BAD_ARGUMENT){
        return ("Bad Argument");
    } else if(_error == UPDATE_ERROR_ABORT){
        return ("Aborted");
    }
    return ("UNKNOWN");
}

/**
 * @brief Read data from encrypted partition using Memory Mapper
 * 
 * esp_partition_read is not usable for encrypted partitions
 * 
 * @param partition Partition to read data from
 * @param offset Offset of the data to read in the partition
 * @param buffer Buffer to store read data (should be at least `len` bytes long)
 * @param len Amount of data to read from the partition
 */
static bool _readPartitionData(const esp_partition_t* partition, size_t offset, uint8_t *buffer, size_t len)
{
    if (!partition || (!buffer) || (partition->size<(offset+len)))
        return false;

    spi_flash_mmap_handle_t ota_data_mmap;
    const void* result = NULL;
    esp_err_t err = esp_partition_mmap(partition, 0, partition->size, SPI_FLASH_MMAP_DATA, &result, &ota_data_mmap);

    if (err != ESP_OK)
    {
        return false;
    } else {
        memcpy(buffer, (uint8_t*)result+offset, len);
        spi_flash_munmap(ota_data_mmap);
    }
    return true;
}

static bool _partitionIsBootable(const esp_partition_t* partition){
    uint8_t buf[8];
    memset(buf, 0, 4);
    if(!partition){
        return false;
    }
    if(!_readPartitionData(partition, 0, buf, 8)){
        return false;
    }

    if(buf[0] != ESP_IMAGE_HEADER_MAGIC) {
        return false;
    }
    return true;
}

static bool _enablePartition(const esp_partition_t* partition){
    uint8_t *buf;
    if(!partition){
        return false;
    }

    buf = (uint8_t*)malloc(SPI_FLASH_SEC_SIZE);

    if(buf == NULL)
        return false;

    if(!_readPartitionData(partition, 0, buf, SPI_FLASH_SEC_SIZE)){
        free(buf);
        return false;
    }

    buf[0] = ESP_IMAGE_HEADER_MAGIC;

    esp_partition_erase_range(partition, 0, SPI_FLASH_SEC_SIZE);
    if (esp_partition_write(partition, 0, buf, SPI_FLASH_SEC_SIZE) != ESP_OK)
    {
        free(buf);
        return false;
    }

    free(buf);
    return true;
}

UpdateClass::UpdateClass()
: _error(0)
, _buffer(0)
, _bufferLen(0)
, _size(0)
, _progress_callback(NULL)
, _progress(0)
, _command(U_FLASH)
, _partition(NULL)
, partialBytes(0)
, partialData()
{
}

UpdateClass& UpdateClass::onProgress(THandlerFunction_Progress fn) {
    _progress_callback = fn;
    return *this;
}

void UpdateClass::_reset() {
    if (_buffer)
        delete[] _buffer;
    _buffer = 0;
    _bufferLen = 0;
    _progress = 0;
    _size = 0;
    _command = U_FLASH;

    if(_ledPin != -1) {
      digitalWrite(_ledPin, !_ledOn); // off
    }
}

bool UpdateClass::canRollBack(){
    if(_buffer){ //Update is running
        return false;
    }
    const esp_partition_t* partition = esp_ota_get_next_update_partition(NULL);
    return _partitionIsBootable(partition);
}

bool UpdateClass::rollBack(){
    if(_buffer){ //Update is running
        return false;
    }
    const esp_partition_t* partition = esp_ota_get_next_update_partition(NULL);
    return _partitionIsBootable(partition) && !esp_ota_set_boot_partition(partition);
}

bool UpdateClass::begin(size_t size, int command, int ledPin, uint8_t ledOn) {
    if(_size > 0){
        log_w("already running");
        return false;
    }

    _ledPin = ledPin;
    _ledOn = !!ledOn; // 0(LOW) or 1(HIGH)

    _reset();
    _error = 0;

    if(size == 0) {
        _error = UPDATE_ERROR_SIZE;
        return false;
    }

    if (command == U_FLASH) {
        _partition = esp_ota_get_next_update_partition(NULL);
        if(!_partition){
            _error = UPDATE_ERROR_NO_PARTITION;
            return false;
        }
        log_d("OTA Partition: %s", _partition->label);
    }
    else if (command == U_SPIFFS) {
        _partition = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_DATA_SPIFFS, NULL);
        if(!_partition){
            _error = UPDATE_ERROR_NO_PARTITION;
            return false;
        }
    }
    else {
        _error = UPDATE_ERROR_BAD_ARGUMENT;
        log_e("bad command %u", command);
        return false;
    }

    if(size == UPDATE_SIZE_UNKNOWN){
        size = _partition->size;
    } else if(size > _partition->size){
        _error = UPDATE_ERROR_SIZE;
        log_e("too large %u > %u", size, _partition->size);
        return false;
    }

    //initialize
    _buffer = (uint8_t*)malloc(SPI_FLASH_SEC_SIZE);
    if(!_buffer){
        log_e("malloc failed");
        return false;
    }
    _size = size;
    _command = command;
    _md5.begin();

    // Erase the partition before starting firmware update
    if (esp_partition_erase_range(_partition, 0, (_size / SPI_FLASH_SEC_SIZE + 1) * SPI_FLASH_SEC_SIZE) != ESP_OK) {
        _abort(UPDATE_ERROR_ERASE);
        return false;
    }
    return true;
}

void UpdateClass::_abort(uint8_t err){
    _reset();
    _error = err;
}

void UpdateClass::abort(){
    _abort(UPDATE_ERROR_ABORT);
}

/**
 * @brief Write buffer content to the end of the partition
 * 
 * This function has been rewritten according to esp_ota_write procedure to handle
 * multiple of 16 bytes block write, and seemless data encryption using esp_partition_write function.
 * 
 * @param final Indicate the end of transfert, forces the write of remaining data to the end of flash
 **/
bool UpdateClass::_writeBuffer(bool final){
    int copyLen = 0;
    //first bytes of new firmware
    if(!_progress && _command == U_FLASH){
        //check magic
        if(_buffer[0] != ESP_IMAGE_HEADER_MAGIC){
            _abort(UPDATE_ERROR_MAGIC_BYTE);
            return false;
        }
        //remove magic byte from the firmware now and write it upon success
        //this ensures that partially written firmware will not be bootable
        _buffer[0] = 0xFF;
    }

    if (!_progress && _progress_callback) {
        _progress_callback(0, _size);
    }
    
    // Check if there are still data to write from last write
    if (this->partialBytes > 0)
    {
        // Complete buffer with data from the beginning of current buffer
        copyLen = MIN(16 - this->partialBytes, _bufferLen);
        memcpy(this->partialData + this->partialBytes, _buffer, copyLen);
        this->partialBytes += copyLen;

        // If enouth data to write one block
        if (this->partialBytes == 16)
        {
            // Write the block
            if (esp_partition_write(_partition, _progress, this->partialData, 16) != ESP_OK ) {
                _abort(UPDATE_ERROR_WRITE);
                return false;
            }

            // Clear the buffer
            this->partialBytes = 0;
            _md5.add(this->partialData, 16);
            memset(this->partialData, 0xFF, 16);
            // And update the datalength
            _progress += 16;
            _bufferLen -= copyLen;
        }            
    }

    // check if more data to store
    this->partialBytes = _bufferLen % 16;
    if (this->partialBytes != 0)
    {
        _bufferLen -= this->partialBytes;
        memcpy(this->partialData, _buffer + copyLen + _bufferLen, this->partialBytes);
    }

    // Write completes blocks from data
    if (esp_partition_write(_partition, _progress, _buffer + copyLen, _bufferLen) != ESP_OK ) {
        _abort(UPDATE_ERROR_WRITE);
        return false;
    }

    //restore magic or md5 will fail
    if(!_progress && _command == U_FLASH){
        _buffer[0] = ESP_IMAGE_HEADER_MAGIC;
    }
    
    // Compute the MD5 sum
    _md5.add(_buffer + copyLen, _bufferLen);
    _progress += _bufferLen;
    _bufferLen = 0;

    // Forces final block write if data remaining in buffer
    if (final && (partialBytes>0))
    {
        log_d("Writting final %d bytes", partialBytes);
        if (esp_partition_write(_partition, _progress, this->partialData, 16) != ESP_OK ) {
            _abort(UPDATE_ERROR_WRITE);
            return false;
        }
        _md5.add(this->partialData, this->partialBytes);
        _progress += this->partialBytes;
    }

    if (_progress_callback) {
        _progress_callback(_progress, _size);
    }
    return true;
}

bool UpdateClass::_verifyHeader(uint8_t data) {
    if(_command == U_FLASH) {
        if(data != ESP_IMAGE_HEADER_MAGIC) {
            _abort(UPDATE_ERROR_MAGIC_BYTE);
            return false;
        }
        return true;
    } else if(_command == U_SPIFFS) {
        return true;
    }
    return false;
}

bool UpdateClass::_verifyEnd() {
    if(_command == U_FLASH) {
        if(!_enablePartition(_partition) || !_partitionIsBootable(_partition)) {
            _abort(UPDATE_ERROR_READ);
            return false;
        }

        if(esp_ota_set_boot_partition(_partition)){
            _abort(UPDATE_ERROR_ACTIVATE);
            return false;
        }
        _reset();
        return true;
    } else if(_command == U_SPIFFS) {
        _reset();
        return true;
    }
    return false;
}

bool UpdateClass::setMD5(const char * expected_md5){
    if(strlen(expected_md5) != 32)
    {
        return false;
    }
    _target_md5 = expected_md5;
    return true;
}

bool UpdateClass::end(bool evenIfRemaining){
    if(hasError() || _size == 0){
        return false;
    }

    if(!isFinished() && !evenIfRemaining){
        log_e("premature end: res:%u, pos:%u/%u\n", getError(), progress(), _size);
        _abort(UPDATE_ERROR_ABORT);
        return false;
    }

    if(evenIfRemaining) {
        if(_bufferLen > 0) {
            _writeBuffer(true);
        }
        _size = progress();
    }

    _md5.calculate();
    if(_target_md5.length()) {
        if(_target_md5 != _md5.toString()){
            _abort(UPDATE_ERROR_MD5);
            return false;
        }
    }

    return _verifyEnd();
}

size_t UpdateClass::write(uint8_t *data, size_t len) {
    if(hasError() || !isRunning()){
        return 0;
    }

    if(len > remaining()){
        _abort(UPDATE_ERROR_SPACE);
        return 0;
    }

    size_t left = len;

    while((_bufferLen + left) > SPI_FLASH_SEC_SIZE) {
        size_t toBuff = SPI_FLASH_SEC_SIZE - _bufferLen;
        memcpy(_buffer + _bufferLen, data + (len - left), toBuff);
        _bufferLen += toBuff;
        if(!_writeBuffer()){
            return len - left;
        }
        left -= toBuff;
    }
    memcpy(_buffer + _bufferLen, data + (len - left), left);
    _bufferLen += left;
    if(_bufferLen == remaining()){
        if(!_writeBuffer(true)){
            return len - left;
        }
    }
    return len;
}

size_t UpdateClass::writeStream(Stream &data) {
    size_t written = 0;
    size_t toRead = 0;
    if(hasError() || !isRunning())
        return 0;

    if(!_verifyHeader(data.peek())) {
        _reset();
        return 0;
    }

    if(_ledPin != -1) {
        pinMode(_ledPin, OUTPUT);
    }

    while(remaining()) {
        if(_ledPin != -1) {
            digitalWrite(_ledPin, _ledOn); // Switch LED on
        }
        size_t bytesToRead = SPI_FLASH_SEC_SIZE - _bufferLen;
        if(bytesToRead > remaining()) {
            bytesToRead = remaining();
        }

        toRead = data.readBytes(_buffer + _bufferLen,  bytesToRead);
        if(toRead == 0) { //Timeout
            delay(100);
            toRead = data.readBytes(_buffer + _bufferLen, bytesToRead);
            if(toRead == 0) { //Timeout
                _abort(UPDATE_ERROR_STREAM);
                return written;
            }
        }
        if(_ledPin != -1) {
            digitalWrite(_ledPin, !_ledOn); // Switch LED off
        }
        _bufferLen += toRead;
        if((_bufferLen == remaining() || _bufferLen == SPI_FLASH_SEC_SIZE) && !_writeBuffer())
            return written;
        written += toRead;
    }
    return written;
}

void UpdateClass::printError(Stream &out){
    out.println(_err2str(_error));
}

const char * UpdateClass::errorString(){
    return _err2str(_error);
}

UpdateClass Update;
