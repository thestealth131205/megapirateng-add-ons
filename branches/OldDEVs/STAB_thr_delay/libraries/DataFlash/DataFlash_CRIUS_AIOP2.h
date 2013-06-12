/* ************************************************************ */
/* Test for DataFlash Log library                               */
/* ************************************************************ */
#ifndef __DataFlash_CRIUS_AIOP2_H__
#define __DataFlash_CRIUS_AIOP2_H__

#include "DataFlash.h"

class DataFlash_CRIUS_AIOP2 : public DataFlash_Class
{
  private:
	//Methods
	unsigned char BufferRead (unsigned char BufferNum, uint16_t IntPageAdr);
	void BufferWrite (unsigned char BufferNum, uint16_t IntPageAdr, unsigned char Data);
	void BufferToPage (unsigned char BufferNum, uint16_t PageAdr, unsigned char wait);
	void PageToBuffer(unsigned char BufferNum, uint16_t PageAdr);
	void WaitReady();
	unsigned char ReadStatusReg();
	unsigned char ReadStatus();
	uint16_t PageSize();
	void PageErase (uint16_t PageAdr);
	void BlockErase (uint16_t BlockAdr);
	void ChipErase(void (*delay_cb)(unsigned long));

  public:

	DataFlash_CRIUS_AIOP2(); // Constructor
	void Init();
	void ReadManufacturerID();
	bool CardInserted(void);
};

#endif // __DataFlash_CRIUS_AIOP2_H__
