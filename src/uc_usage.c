/**
 * @file	uc_usage.c
 * @brief  	Functions to show the uC usage
 * 
 * @sources			http://www.chibios.com/forum/viewtopic.php?f=2&t=138&start=10
 * 					http://www.chibios.com/forum/viewtopic.php?t=4496
 * @modified by  	Eliot Ferragni
 */
 
#include "main.h"

//////////////////////////////////////////PUBLIC FUNCTIONS/////////////////////////////////////////

void printUcUsage(BaseSequentialStream* out) {
	
	thread_t *tp;

	static uint64_t sum;
	static uint16_t tmp1, tmp2;
	sum = 0;
	//takes the first thread created
	tp = chRegFirstThread();
	//computes the total number of cycles counted
	do {
		sum += tp->stats.cumulative;
		tp = chRegNextThread(tp);
	} while (tp != NULL);
	sum += ch.kernel_stats.m_crit_thd.cumulative;
	sum += ch.kernel_stats.m_crit_isr.cumulative;

	//takes the first thread created
	tp = chRegFirstThread();
	//computes the percentage of time used by eahc thread
	do {
		tmp1 = (uint16_t)(tp->stats.cumulative*10000/sum);
		chprintf(out, "%30s %u.%u%%\r\n", tp->name, tmp1/100, tmp1%100);
		tp = chRegNextThread(tp);
	} while (tp != NULL);

	tmp1 = (uint16_t)(ch.kernel_stats.m_crit_thd.cumulative*10000/sum);
	tmp2 = (uint16_t)(ch.kernel_stats.m_crit_isr.cumulative*10000/sum);

	chprintf(out, "critical thd:%u.%u%%   critical isr:%u.%u%%\r\n",
	  tmp1/100, tmp1%100,tmp2/100, tmp2%100);
	chprintf(out, "\r\n");
}

void printStatThreads(BaseSequentialStream *out)
{
  static const char *states[] = {CH_STATE_NAMES};
  thread_t *tp;
  size_t n = 0;
  size_t sz;
  uint32_t used_pct;

  chprintf(out, "\r\n");
  chprintf(out, "     begin        end   size   used    %% prio     state         name\r\n");
  chprintf(out, "--------------------------------------------------------------------\r\n");

  tp = chRegFirstThread();
  do {
     n = 0;
#if (CH_DBG_ENABLE_STACK_CHECK == TRUE) || (CH_CFG_USE_DYNAMIC == TRUE)
    uint32_t stklimit = (uint32_t)tp->wabase;
#else
    uint32_t stklimit = 0U;
#endif
    uint8_t *begin = (uint8_t *)stklimit;
    uint8_t *end = (uint8_t *)tp;
    sz = end - begin;

    while(begin < end)
       if(*begin++ == CH_DBG_STACK_FILL_VALUE) ++n;

    used_pct = (n * 100) / sz;

    chprintf(out, "0x%08lx 0x%08lx %6u %6u %3u%% %4lu %9s %12s\r\n", stklimit,                                                                                           
    																 (uint32_t)tp,
                                                                     sz,
                                                                     n,
                                                                     used_pct,
                                                                     (uint32_t)tp->prio,
                                                                     states[tp->state],
                                                                     tp->name == NULL ? "" : tp->name);

    tp = chRegNextThread(tp);
  } while (tp != NULL);

  chprintf(out, "\r\n");
}