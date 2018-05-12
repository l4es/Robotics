
#ifndef  COLORSPACECONVERSIONS_INC
#define  COLORSPACECONVERSIONS_INC

class ColorSpaceUtils
{
	public:
	static void convertFromYUVToRGB(
		unsigned char Y,
		unsigned char U,//Cb
		unsigned char V,//Cr
		unsigned char& R,
		unsigned char& G,
		unsigned char& B)
	{
		int r = (Y + ((1436*( V - 128)) >> 10)),
		g     = (Y - (( 732*( V - 128) + (354*(U - 128))) >> 10)),
		b     = (Y + ((1814*( U - 128)) >> 10));
		if(r < 0) r = 0; else if(r > 255) r = 255;
		if(g < 0) g = 0; else if(g > 255) g = 255;
		if(b < 0) b = 0; else if(b > 255) b = 255;
		R = (unsigned char) r;
		G = (unsigned char) g;
		B = (unsigned char) b;
	}

	static void convertFromRGBToYUV(
		unsigned char R,
		unsigned char G,
		unsigned char B,
		unsigned char& Y,
		unsigned char& U,
		unsigned char& V)
	{
		int y = (int)( 0.2990 * R + 0.5870 * G + 0.1140 * B),
				u = 127 + (int)( 0.5000 * R - 0.4187 * G - 0.0813 * B),
				v = 127 + (int)(-0.1687 * R - 0.3313 * G + 0.5000 * B);
		if(y < 0) y = 0; else if(y > 255) y = 255;
		if(u < 0) u = 0; else if(u > 255) u = 255;
		if(v < 0) v = 0; else if(v > 255) v = 255;
		Y = (unsigned char) y;
		U = (unsigned char) u;
		V = (unsigned char) v;
	}
};



#endif   /* ----- #ifndef COLORSPACECONVERSIONS_INC  ----- */
