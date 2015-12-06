#include <dsp.h>

void pid_init(pid_instance * S, int32 resetStateFlag)
{

	/* Derived coefficient A0 */
	S->A0 = S->Kp + S->Ki + S->Kd;

	/* Derived coefficient A1 */
	S->A1 = (-S->Kp) - ((float) 2.0 * S->Kd);

	/* Derived coefficient A2 */
	S->A2 = S->Kd;

	/* Check whether state needs reset or not */
	if (resetStateFlag)
	{
		/* Clear the state buffer.  The size will be always 3 samples */
		memset_fast(S->state, 0, 3u * 2);
	}

}

void pid_reset(pid_instance * S)
{

	/* Clear the state buffer.  The size will be always 3 samples */
	memset_fast(S->state, 0, 3u * 2);
}

/**    
 * @} end of PID group    
 */
