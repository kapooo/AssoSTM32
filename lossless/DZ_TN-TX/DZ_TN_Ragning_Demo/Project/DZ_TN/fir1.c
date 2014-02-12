/*******************************************************************************
* Function Name  : single_point_FIR
* Input          : short* x, short* h, int taps
* Output         : None
* Return         : The 32-bit signed filtered data point
* Description    : 

Performs a single point FIR of the data pointed to by x, using the filter coefficients
pointed to by h.  'taps' is the number of taps used in the filter.  This function
call assumes that the filter coefficients are symmetric, so that only half the
coefficients are stored in the memory location pointed to by h.  The size of the
buffer pointed to by x must be equal to the number of taps in the filter.

x and h are 16-bit signed, fixed-point data types.  To obtain the 16-bit version
of the filter output, the output should be multiplied by two, and then right-shifted
by 16.

*******************************************************************************/
int single_point_FIR( short* x, short* h, int taps )
{
	 int j, sum, taps_div_2;
	 int h_index, x_index;
	 
	 taps_div_2 = taps >> 1;
	 
	 sum = 0;

	 // Break the convolution sum into two halves, since we only store half of the symmetric 
	 // filter impulse response.
	 for( j=0; j < taps_div_2; j++ )
	 {
		  sum += h[j]*x[j];
		  sum += h[taps_div_2 - j - 1]*x[taps_div_2 + j];
	 }	  

	 return sum;
}