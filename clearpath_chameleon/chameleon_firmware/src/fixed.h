#ifndef FIXED_H
#define FIXED_H
/* 
 * Fixed point number class
 * Copyright (C) 2011 Mike Purvis <clearpathrobotics.com>
 * Copyright (C) 2008 Dave Robillard <drobilla.net>
 * Copyright (C) 2006 Henry Strickland & Ryan Seto
 *
 * <http://www.opensource.org/licenses/mit-license.php>
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR
 * OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE,
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR
 * OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdint.h>

#define INT16_MAX   0x7fff
#define INT16_MIN   (-INT16_MAX - 1)


class Fixed {
public:
	Fixed() : g(0) {}
	Fixed(const Fixed& a) : g( a.g ) {}
	Fixed(float a) : g( (int16_t)(a / (float)STEP) ) {}
	Fixed(double a) : g( (int16_t)(a / (double)STEP) ) {}
	Fixed(int a) : g( a << F ) {}
	Fixed(long a) : g( a << F ) {}
	Fixed& operator =(const Fixed& a) { g= a.g; return *this; }
	Fixed& operator =(float a) { g= Fixed(a).g; return *this; }
	Fixed& operator =(double a) { g= Fixed(a).g; return *this; }
	Fixed& operator =(int a) { g= Fixed(a).g; return *this; }
	Fixed& operator =(long a) { g= Fixed(a).g; return *this; }

        operator float()  { return g * (float)STEP; }
	operator int()    { return g>>F; }
	operator long()   { return g>>F; }

	Fixed operator +() const { return Fixed(RAW,g); }
	Fixed operator -() const { return Fixed(RAW,-g); }

//	Fixed operator +(const Fixed& a) const { return Fixed(RAW, g + a.g); }
        // Saturating adder
        Fixed operator +(const Fixed& a) const {
          int32_t result = (int32_t)g + (int32_t)a.g; 
          if (result > INT16_MAX)
            // saturated
            return Fixed(RAW, INT16_MAX);
          else if (result < INT16_MIN)
            return Fixed(RAW, INT16_MIN);
          else
            return Fixed(RAW, (int16_t)result);
        }
        
	Fixed operator -(const Fixed& a) const { return Fixed(RAW, g - a.g); }
//	Fixed operator *(const Fixed& a) const { return Fixed(RAW,  (int16_t)( ((int32_t)g * (int32_t)a.g ) >> F)); }

        // Saturating multipier.
	Fixed operator *(const Fixed& a) const {
          int32_t result = ((int32_t)g * (int32_t)a.g ) >> F;
          if (result > INT16_MAX)
            // saturated
            return Fixed(RAW, INT16_MAX);
          else if (result < INT16_MIN)
            return Fixed(RAW, INT16_MIN);
          else
            return Fixed(RAW, (int16_t)result);      
        }

	Fixed operator /(const Fixed& a) const { return Fixed(RAW, (int16_t)( (((int32_t)g << F2) / (int32_t)(a.g)) >> F) ); }

	Fixed& operator +=(Fixed a) { return *this = *this + a; return *this; }
	Fixed& operator -=(Fixed a) { return *this = *this - a; return *this; }
	Fixed& operator *=(Fixed a) { return *this = *this * a; return *this; }
	Fixed& operator /=(Fixed a) { return *this = *this / a; return *this; }

        bool operator ==(const Fixed& a) const { return g == a.g; }
        bool operator !=(const Fixed& a) const { return g != a.g; }
	bool operator <=(const Fixed& a) const { return g <= a.g; }
	bool operator >=(const Fixed& a) const { return g >= a.g; }
	bool operator  <(const Fixed& a) const { return g  < a.g; }
	bool operator  >(const Fixed& a) const { return g  > a.g; }

        uint8_t get_byte(uint8_t i) const { return bytes[i]; }

        static Fixed from_raw(int16_t raw) { return Fixed(RAW, raw); }
private:
        union {
	        int16_t g;
                uint8_t bytes[];
        };
        
	const static uint8_t F     = 8;
	const static uint8_t F2    = F * 2;
	const static uint8_t Fhalf = F/2;

	/** Smallest step we can represent */
        const static float STEP = 1.0 / (1 << F);
        
	/** Private direct constructor from 'guts' */
	enum FixedRaw { RAW };
	Fixed(FixedRaw, int16_t guts) : g(guts) {}        
};

#endif
