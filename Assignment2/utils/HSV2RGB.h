/******************************************************************
*
* HSV2RGB.h
*
* Description: Convert from HSV to RGB color space
*
* Physically-Based Simulation Proseminar WS 2015
*
* Interactive Graphics and Simulation Group
* Institute of Computer Science
* University of Innsbruck
*
*******************************************************************/

#ifndef __HSV2RGB_T_H__
#define __HSV2RGB_T_H__

void HSV2RGB(double h, double s, double v, double &r, double &g, double &b)
{
    h /= 360.0;
    if(fabs(h - 1.0) < 0.000001)
        h = 0.0;

    h *= 6.0;
    
    const auto i = static_cast<int>(floor(h));

    double f = h - i;
    double p = v * (1.0 - s);
    double q = v * (1.0 - (s * f));
    double t = v * (1.0 - (s * (1.0 - f)));
    
    switch (i)
    {
        case 0: r=v; g=t; b=p; break;
        case 1: r=q; g=v; b=p; break;
        case 2: r=p; g=v; b=t; break;
        case 3: r=p; g=q; b=v; break;
        case 4: r=t; g=p; b=v; break;
        case 5: r=v; g=p; b=q; break;
    }
}

#endif
