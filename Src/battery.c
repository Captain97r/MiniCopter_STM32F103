#include "battery.h"


#define STOPPER				0
#define MEDIAN_FILTER_SIZE	5
#define ADC_FULL_VALUE		4095
#define VDD_VOLTAGE			3.3

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	median = MedianFilter((uint16_t)battery.buf[0]);
}

void battery_calculate_voltage()
{
	float vref = VDD_VOLTAGE * battery.buf[1] / ADC_FULL_VALUE;
	float vbat = 2.65 * 2.0 * vref * median / ADC_FULL_VALUE;
	battery.voltage_mult_1000 = (uint16_t)(vbat * 1000);
	battery.data_rdy = 1;
}

uint16_t get_battery_voltage()
{
	return battery.voltage_mult_1000;
}


uint16_t MedianFilter(uint16_t datum)
{
	struct pair {
		struct pair *point; /* Pointers forming list linked in sorted order */
		uint16_t value; /* Values to sort */
	};
 
	/* Buffer of nwidth pairs */
	static struct pair buffer[MEDIAN_FILTER_SIZE] = { 0 };
	/* Pointer into circular buffer of data */
	static struct pair *datpoint = buffer; 
	/* Chain stopper */
	static struct pair small = { NULL, STOPPER };
	/* Pointer to head (largest) of linked list.*/
	static struct pair big = { &small, 0 }; 

	/* Pointer to successor of replaced data item */
	struct pair *successor; 
	/* Pointer used to scan down the sorted list */
	struct pair *scan; 
	/* Previous value of scan */
	struct pair *scanold;
	/* Pointer to median */
	struct pair *median; 
	uint16_t i;

	if (datum == STOPPER) {
		datum = STOPPER + 1; /* No stoppers allowed. */
	}

	if ((++datpoint - buffer) >= MEDIAN_FILTER_SIZE) {
		datpoint = buffer; /* Increment and wrap data in pointer.*/
	}

	datpoint->value = datum; /* Copy in new datum */
	successor = datpoint->point; /* Save pointer to old value's successor */
	median = &big; /* Median initially to first in chain */
	scanold = NULL; /* Scanold initially null. */
	scan = &big; /* Points to pointer to first (largest) datum in chain */

	/* Handle chain-out of first item in chain as special case */
	if (scan->point == datpoint) {
		scan->point = successor;
	}

	scanold = scan; /* Save this pointer and */
	scan = scan->point; /* step down chain */

	/* Loop through the chain, normal loop exit via break. */
	for (i = 0; i < MEDIAN_FILTER_SIZE; ++i) {
		/* Handle odd-numbered item in chain */
		if (scan->point == datpoint) {
			scan->point = successor; /* Chain out the old datum.*/
		}

		if (scan->value < datum) {
			/* If datum is larger than scanned value,*/
			datpoint->point = scanold->point; /* Chain it in here. */
			scanold->point = datpoint; /* Mark it chained in. */
			datum = STOPPER;
		}
		;

		/* Step median pointer down chain after doing odd-numbered element */
		median = median->point; /* Step median pointer. */
		if (scan == &small) {
			break; /* Break at end of chain */
		}
		scanold = scan; /* Save this pointer and */
		scan = scan->point; /* step down chain */

		/* Handle even-numbered item in chain. */
		if (scan->point == datpoint) {
			scan->point = successor;
		}

		if (scan->value < datum) {
			datpoint->point = scanold->point;
			scanold->point = datpoint;
			datum = STOPPER;
		}

		if (scan == &small) {
			break;
		}

		scanold = scan;
		scan = scan->point;
	}
 
	return median->value;
}