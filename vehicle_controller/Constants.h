/*==========================================================================*/
/*  ColorConstants.h													    */
/*  Version of 2021-06-08                             Fernando V. Monteiro  */
/*==========================================================================*/

/* 
Note: I tried but couldn't use enum class instead of the macros below. 
I couldn't make the enum class Color recognize the COLORREF, BYTE etc types
I couldn't make the enum class Category use long (to match VISSIM's type)

TODO: change to constexpr? Put them in a struct or class?
*/

#pragma once
/* Based on the _WINGDI_ RGB definition and some tests on VISSIM */
//#define ARGB(a,r,g,b)   ((COLORREF)((((BYTE)(b) | ((WORD)((BYTE)(g)) << 8)) | (((DWORD)(BYTE)(r)) << 16))|((BYTE)(a) << 24)))
#define ARGB(a, r, g, b) ((unsigned long)((long)(b) + (long)(g)*256 + (long)(r)*256*256 + (unsigned long)(a)*256*256*256))

enum class VehicleColor : unsigned long {
	red = ARGB(255, 255, 0, 0),
	green = ARGB(255, 0, 255, 0),
	blue = ARGB(255, 0, 0, 255),
	yellow = ARGB(255, 255, 255, 96),
	black = ARGB(255, 0, 0, 0),
	white = ARGB(255, 255, 255, 255),
};
//#define RED		ARGB(255, 255, 0, 0)
//#define GREEN	ARGB(255, 0, 255, 0)
//#define BLUE	ARGB(255, 0, 0, 255)
//#define YELLOW	ARGB(255, 255, 255, 96)
//#define BLACK	ARGB(255, 0, 0, 0)
//#define WHITE	ARGB(255, 255, 255, 255)

#define CAR 1
#define TRUCK 2
#define BUS 3
#define TRAM 4
#define PEDESTRIAN 5
#define BIKE 6