#include <math.h>

#define ORDER   (4)

static const float coefficient[ORDER] =
{
    0.047f, 0.453f, 0.453f, 0.047f
};
static float buffer[ORDER] = { 0, 0, 0, 0 };

float ProcessSample( float sample )
{
    int i, j;
    /* Process the transpose form of FIR */
    j = 0;
    for( i = ORDER - 1; i > 0; i-- )
    {
        buffer[i] = buffer[i - 1] + ( sample * coefficient[j] );
        j++;
    }
    buffer[0] = sample * coefficient[ORDER - 1];

    return( buffer[ORDER - 1] );
}

float GetOutput()
{
    return( buffer[ORDER - 1] );
}

void InitFIR()
{
    for(int i = 0; i < ORDER; i++ )
        buffer[i] = 0;
}
